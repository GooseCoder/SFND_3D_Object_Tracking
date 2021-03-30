
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;

        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    std::vector<cv::DMatch> kpsWithinRoi;

    // match keypoints with the camera image.
    for (auto it=kptMatches.begin(); it!=kptMatches.end(); ++it)
    {
        cv::KeyPoint query = kptsPrev.at(it->queryIdx); 
        auto queryPoint = cv::Point(query.pt.x, query.pt.y);

        cv::KeyPoint train = kptsCurr.at(it->trainIdx);
        auto trainPoint = cv::Point(train.pt.x, train.pt.y);

        if (boundingBox.roi.contains(trainPoint) && boundingBox.roi.contains(queryPoint)) {
            kpsWithinRoi.push_back(*it);
        }
    }

    // eliminates outlier matches with distances larger than the mean.
    double meanDistance = 0;
    for (auto it=kpsWithinRoi.begin(); it!=kpsWithinRoi.end(); ++it)
    {
        meanDistance += cv::norm(kptsCurr.at(it->trainIdx).pt - kptsPrev.at(it->queryIdx).pt); 
    }
    meanDistance /= kpsWithinRoi.size();

    // populate bounding box
    for (auto it = kpsWithinRoi.begin(); it!=kpsWithinRoi.end(); ++it)
    {
       float distance = cv::norm(kptsCurr.at(it->trainIdx).pt - kptsPrev.at(it->queryIdx).pt);
       if (distance < 1.3*meanDistance) {
           boundingBox.kptMatches.push_back(*it);
       }
    }
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // calculate ratio distance between keypoints matched 
    vector<double> distanceRatios; 
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { 
        // get and store current keypoint
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        {
            double minDistance = 100.0;

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);

            // compute distances and distance ratios
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDistance)
            {
                double distRatio = distCurr / distPrev;
                distanceRatios.push_back(distRatio);
            }
        }
    }

    // check if only the list of distance ratios has elements
    if (distanceRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    std::sort(distanceRatios.begin(), distanceRatios.end());
    long mediumIndex = floor(distanceRatios.size() / 2.0);
    double medDistanceRatio = distanceRatios.size() % 2 == 0 ? (distanceRatios[mediumIndex - 1] + distanceRatios[mediumIndex]) / 2.0 : distanceRatios[mediumIndex]; // compute median distance. ratio to remove outlier influence

    TTC = -(1/frameRate) / (1 - medDistanceRatio);
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // take into account only Lidar points within ego lane
    std::vector<LidarPoint> filteredLidarPointsCurr;
    std::vector<LidarPoint> filteredLidarPointsPrev;

    for(auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        if(abs(it->y) < 2) 
            filteredLidarPointsPrev.push_back(*it);
    }
    for(auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        if(abs(it->y) < 2) 
            filteredLidarPointsCurr.push_back(*it);
    }

    double avgPrev, avgCurr;
    double d_0 = 1e9, d_1 = 1e9;

    for (auto it=filteredLidarPointsPrev.begin(); it!=filteredLidarPointsPrev.end(); ++it) {
        avgPrev += it->x;
    }
    d_0 = avgPrev / filteredLidarPointsPrev.size();

    for (auto it=filteredLidarPointsCurr.begin(); it!=filteredLidarPointsCurr.end(); ++it) {
        avgCurr += it->x;
    }
    d_1 = avgCurr / filteredLidarPointsCurr.size();

    TTC = (d_1 * (1/frameRate)) / (d_0 - d_1);
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    int pointCount[10][10] = {{0}};
    for (auto it = matches.begin(); it != matches.end() - 1; ++it)     
    {
        cv::KeyPoint query = prevFrame.keypoints[it->queryIdx];
        bool queryFound = false;
        auto queryPoint = cv::Point(query.pt.x, query.pt.y);
        
        cv::KeyPoint train = currFrame.keypoints[it->trainIdx];
        bool trainFound = false;
        auto trainPoint = cv::Point(train.pt.x, train.pt.y);


        std::vector<int> query_id, train_id;
        for (int i = 0; i < 10; i++) 
        {
            if (prevFrame.boundingBoxes[i].roi.contains(queryPoint))            
            {
                queryFound = true;
                query_id.push_back(i);
            }
            if (currFrame.boundingBoxes[i].roi.contains(trainPoint))            
            {
                trainFound = true;
                train_id.push_back(i);
            }
        }

        if (queryFound && trainFound) 
        {
            for (int i: query_id) {
                for (int j: train_id) {
                    pointCount[i][j] += 1;
                }
            }
        }
    }
   
    //for each row in the array, find the best column (best = most counts)
    for (int i=0; i<10; i++) {
        int row_max = 0;
        int best_col_idx = 0;
        for (int j=0; j<10; j++) {
            if (pointCount[i][j] > row_max) {
                row_max = pointCount[i][j];
                best_col_idx = j;
            }
        }
        bbBestMatches[i] = best_col_idx;
    }
}
