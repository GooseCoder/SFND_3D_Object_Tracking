# SFND 3D Object Tracking

Welcome to the final project of the camera course. By completing all the lessons, you now have a solid understanding of keypoint detectors, descriptors, and methods to match them between successive images. Also, you know how to detect objects in an image using the YOLO deep-learning framework. And finally, you know how to associate regions in a camera image with Lidar points in 3D space. Let's take a look at our program schematic to see what we already have accomplished and what's still missing.

<img src="images/course_code_structure.png" width="779" height="414" />

In this final project, you will implement the missing parts in the schematic. To do this, you will complete four major tasks: 
1. First, you will develop a way to match 3D objects over time by using keypoint correspondences. 
2. Second, you will compute the TTC based on Lidar measurements. 
3. You will then proceed to do the same using the camera, which requires to first associate keypoint matches to regions of interest and then to compute the TTC based on those matches. 
4. And lastly, you will conduct various tests with the framework. Your goal is to identify the most suitable detector/descriptor combination for TTC estimation and also to search for problems that can lead to faulty measurements by the camera or Lidar sensor. In the last course of this Nanodegree, you will learn about the Kalman filter, which is a great way to combine the two independent TTC measurements into an improved version which is much more reliable than a single sensor alone can be. But before we think about such things, let us focus on your final project in the camera course. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* Git LFS
  * Weight files are handled using [LFS](https://git-lfs.github.com/)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.

## Project description and Write-Up
![Sample](./images/results/result_0001.png)

#### 1. 3D Objects Matching
A matchBoundingBoxes method was implemented to process 3d objects matching.

#### 2. Compute TTC Lidar-based 
Collision time calculated in seconds was implemented based on the lidar points within the region of interest of each matched Bounding-Box.

#### 3. Keypoint Correspondences and Bounding Boxes association
Keypoints for previous and current frame are matched / checked if they are within the right ROI. Outliers are being removed with a comparison with the average eucliean mean distance.

#### 4. Compute Camera-based TTC
Time to collision is measured in seconds and is implemented using the relationship between previous and current frames keypoint correspondences.

#### 5. Performance Evaluation 1
One simple example to explain when a Lidar TTC estimate is inplausible is when the lidar sensor returns some points which are clearly not located on the vehicle, but much closer than that. It could be some dust particles in the air caused by weather events (snow, sandstorms) which could introduce noise data into the lidar scans. The result could be a very low TTC.
To prevent this, we need more robust to the outliers by using an average distance instead of a closest point analysis.

One other example of an inplausible Lidar TTC estimate, is where outliers are detected from other parts of the vehicle. For example, the lidar is measuring the mirrors, which are much further away than the actual back of the car. In the experiment, the TTC increased from 12 to 16 seconds even with the minimum distance  has been decreased from 7.78m to 7.69m. These points can be filtered out simply by increasing the shrink factor when clustering the lidar with ROI.

#### 6. Performance Evaluation 2
The combinations of detector/descriptors were tested. The result is that the best performing camera TTC calculations used the AKAZE detector and FREAK descriptor.

One example of an inplausible Camera TTC for this case is if the process takes much more time than 10 seconds. This is could be caused by bad keypoint matching, or that the descriptor is not good enough to recognize keypoints which are clustered close to each other.

| Camera TTC compared to Lidar TTC        | Top performers chart       |
|-----------------------------------------|----------------------------|
| ![Plot](./images/6-alt.png)             | ![Plot](./images/6.png)  |
