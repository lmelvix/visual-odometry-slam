 #include <stdio.h>
 #include <iostream>
 #include <time.h>
 #include <string.h>
 
 #include <opencv2/core/core.hpp>
 #include <opencv2/highgui/highgui.hpp>
 #include <opencv2/imgproc/imgproc.hpp>
 #include <opencv2/features2d/features2d.hpp>
 #include "opencv2/calib3d/calib3d.hpp"
 #include "opencv2/nonfree/nonfree.hpp"
 #include "opencv2/nonfree/features2d.hpp"
 #include <opencv2/video/video.hpp>
 
 #include <boost/thread.hpp>

 #include <Eigen/Core>
 #include <Eigen/Geometry>

 #include "keyframe.hpp"
 #include "frame.hpp"
 #include "draw.hpp"
 #include "basic.hpp"

 #define LEFT_IMAGE "/home/lmelvix/projects/visual-odometry-slam/data/dataset/sequences/08/image_0"
 #define RIGHT_IMAGE "/home/lmelvix/projects/visual-odometry-slam/data/dataset/sequences/08/image_1"
 #define STEREO "/home/lmelvix/projects/visual-odometry-slam/include/stereo252.txt"
 #define matched 1
 #define unmatched 0

 using namespace cv;
 using namespace std;
  
 struct STEREO_RECTIFY_PARAMS{ 
     Mat P1, P2; 
     Mat R1, R2; 
     Mat Q; 
     Mat map11, map12; 
     Mat map21, map22; 
     Point2f leftup; 
     Point2f rightbottom; 
 };
