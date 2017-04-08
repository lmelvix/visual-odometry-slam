//
//  optFlwTrack.hpp
//  sceneReconstruct
//
//  Created by 周晓宇 on 3/16/17.
//  Copyright © 2017 xiaoyu. All rights reserved.
//

#ifndef optFlwTrack_hpp
#define optFlwTrack_hpp

#include <opencv2/core/core.hpp>
//#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include "opencv2/calib3d/calib3d.hpp"

/*
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/common/projection_matrix.h>
*/

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/thread.hpp>

#include <stdio.h>
#include "draw.hpp"
#include "frame.hpp"
#include "keyframe.hpp"


#define matched 1
#define unmatched 0

using namespace cv;
//using namespace pcl;

/*
struct FRAME{
    Mat imgL, imgR;
    vector<KeyPoint> keypointL, keypointR;
    vector<Point2f> p_keypointL, p_keypointR;
//    PointCloud<PointXYZ> ptsCloud;
    Mat desp;
};

struct CAMERA_INTRINSIC{
    float cx, cy;
    float fx, fy;
    Mat dist;
};
struct CAMERA_EXTRINSIC{
    Mat R;
    Mat t;
};
*/

struct STEREO_RECTIFY_PARAMS{
    Mat P1, P2;
    Mat R1, R2;
    Mat Q;
    Mat map11, map12;
    Mat map21, map22;
    Point2f leftup;
    Point2f rightbottom;
    
};

/*

void deleteBadMatch(FRAME& frame, float thres, vector<float> err);
void getDepth(FRAME frame, Mat& inhomo,
              STEREO_RECTIFY_PARAMS srp);

double normofTransform( cv::Mat rvec, cv::Mat tvec );
Mat getCameraProjMat(CAMERA_INTRINSIC K, CAMERA_EXTRINSIC stereo);
Mat getCameraMat(CAMERA_INTRINSIC K);
FRAME getFrame(string filename_left, string filename_right,
               Size imageSize);


void estMotionPnP(FRAME frame1, FRAME frame2, STEREO_RECTIFY_PARAMS srp);


void estMotionPnPOpt(FRAME frame1, FRAME frame2, STEREO_RECTIFY_PARAMS srp);

void estMotionPnPOptFlw(Keyframe keyframe, Frame frame, Mat P2);


void convertMattoPtsCloud(Mat inhomoPts3D, PointCloud<PointXYZ>& ptCloud);
void convert3DHomoToInhomo(Mat homoPts3D, Mat& inhomoPts3D);
*/


#endif /* optFlwTrack_hpp */
