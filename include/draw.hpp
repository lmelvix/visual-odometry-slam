//
//  draw.hpp
//  sceneReconstruct
//
//  Created by 周晓宇 on 4/5/17.
//  Copyright © 2017 xiaoyu. All rights reserved.
//

#ifndef draw_hpp
#define draw_hpp

#include <stdio.h>
#include <opencv2/core/core.hpp>
//#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include "opencv2/calib3d/calib3d.hpp"


using namespace cv;
using namespace std;

void drawFeature(Mat img, vector<Point2f> features, char* windowName);
void drawFeature(Mat img, vector<KeyPoint> features, char* windowName);

void drawMatch(Mat img_1, vector<KeyPoint> keypoints1, vector<KeyPoint> keypoints2, vector<DMatch> matches,int radius);
void drawMatch(Mat img_1, vector<KeyPoint> keypoints1, vector<KeyPoint> keypoints2, int radius);


#endif /* draw_hpp */
