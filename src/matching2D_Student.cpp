#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {
        int normType = cv::NORM_HAMMING;
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        matcher = cv::FlannBasedMatcher::create();
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)
        vector<vector<cv::DMatch>> knnMatches;
        matcher->knnMatch(descSource, descRef, knnMatches, 2);
        // implement the descriptor distance ratio test with t=0.8
        const float threshold = 0.8f;
        for (size_t i=0; i < knnMatches.size(); i++) {
            if (knnMatches[i][0].distance < knnMatches[i][1].distance * threshold) {
                matches.push_back(knnMatches[i][0]);
            }
        }
    }

}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (descriptorType.compare("ORB") == 0) {
        extractor = cv::ORB::create();
    }
    else if (descriptorType.compare("BRIEF") == 0) {
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }
    else if (descriptorType.compare("FREAK") == 0) {
        extractor = cv::xfeatures2d::FREAK::create();
    }
    else if (descriptorType.compare("SIFT") == 0) {
        extractor = cv::xfeatures2d::SIFT::create();
    }
    else if (descriptorType.compare("AKAZE") == 0) {
        extractor = cv::AKAZE::create();
    }

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

// keypointsd detection with harris corners 
void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis) {
    int apertureSize = 3;
    int blockSize = 2;
    double k = 0.04;
    cv::Mat dst;

    double time = (double)cv::getTickCount();
    cv::cornerHarris( img, dst, blockSize, apertureSize, k );
    cv::Mat dstNorm;
    cv::Mat dstNormScaled;
    normalize( dst, dstNorm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
    cv::convertScaleAbs( dstNorm, dstNormScaled );
    int threshold = 100;
    for (int i=0; i < img.rows; ++i) {
        for (int j=0; j < img.cols; ++j) {
            float response = dstNorm.at<float>(i,j);
            if ((int) response > threshold) {
                cv::KeyPoint testKeypoint;
                testKeypoint.pt = cv::Point2f(j, i);
                testKeypoint.size = 2*apertureSize;
                testKeypoint.response = response;

                bool overlap = false;
                for (auto it=keypoints.begin(); it != keypoints.end(); it++) {
                  if (cv::KeyPoint::overlap(testKeypoint, *it) > 0) {
                    overlap = true;
                    if (testKeypoint.response > it->response) {
                      *it = testKeypoint;
                      break;
                    }
                  }
                }
                
                if (!overlap) {
                    keypoints.push_back(testKeypoint);
                }
            }
        }
    }
    time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
    cout << "HarrisCorner: n=" << keypoints.size() << " keypoints, time " << 1000 * time / 1.0 << " (ms)" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = " Results for Harris Corners detection";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis) {
    
    if (detectorType.compare("AKAZE") == 0) {
        cv::Ptr<cv::FeatureDetector> detector = cv::AKAZE::create();
        double time = (double)cv::getTickCount();
        detector->detect(img, keypoints);
        time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
        cout << "AKAZE: n= " << keypoints.size() << " keypoints, time " << 1000 * time / 1.0 << " (ms)" << endl;
    }
    else if (detectorType.compare("SIFT") == 0) {
        cv::Ptr<cv::FeatureDetector> detector = cv::xfeatures2d::SIFT::create();
        double time = (double)cv::getTickCount();
        detector->detect(img, keypoints);
        time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
        cout << "SIFT: n= " << keypoints.size() << " keypoints, time " << 1000 * time / 1.0 << " (ms)" << endl;
    }
    else if (detectorType.compare("ORB") == 0) {
        cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
        double time = (double)cv::getTickCount();
        detector->detect(img, keypoints);
        time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
        cout << "ORB: n= " << keypoints.size() << " keypoints, time " << 1000 * time / 1.0 << " (ms)" << endl;
    }
    else if (detectorType.compare("FAST") == 0) {
        cv::FastFeatureDetector::DetectorType type = cv::FastFeatureDetector::TYPE_7_12;
        cv::Ptr<cv::FeatureDetector> detector = cv::FastFeatureDetector::create(30, true, type);
        double time = (double)cv::getTickCount();
        detector->detect(img, keypoints);
        time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
        cout << "FAST: n= " << keypoints.size() << " keypoints, time " << 1000 * time / 1.0 << " (ms)" << endl;
    }
    else if (detectorType.compare("BRISK") == 0) {
        int threshold = 30;
        int octaves = 3;
        float patternScale = 1.0f;
        cv::Ptr<cv::FeatureDetector> detector = cv::BRISK::create(threshold, octaves, patternScale);
        double time = (double)cv::getTickCount();
        detector->detect(img, keypoints);
        time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
        cout << "BRISK have n= " << keypoints.size() << " keypoints in " << 1000 * time / 1.0 << " ms" << endl;
    }

    if (bVis)
    {
      cv::Mat visImage = img.clone();
      cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
      string windowName = detectorType + " show results...";
      cv::namedWindow(windowName, 2);
      imshow(windowName, visImage);
      cv::waitKey(0);
    }
}
