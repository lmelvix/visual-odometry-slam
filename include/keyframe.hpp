//
//  keyframe.hpp
//  sceneReconstruct
//
//  Created by 周晓宇 on 4/3/17.
//  Copyright © 2017 xiaoyu. All rights reserved.
//

#ifndef keyframe_hpp
#define keyframe_hpp

#include <stdio.h>
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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/point_types.h>
#include <pcl/io/vtk_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/projection_matrix.h>
*/

#include "frame.hpp"
#include "draw.hpp"

using namespace cv;
using namespace std;
//using namespace pcl;

class Keyframe{
    
public:
    
    Mat imgL, imgR;
    vector<KeyPoint> keypointL, keypointR;
    vector<Point2f> p_keypointL, p_keypointR;
    vector<Point3f> scenePts;
    
    vector<int> trackedPtsIdx;//used to store idx of the original feature points, it is used to find original features
    
    vector<Point2f> trackedPts;//store features in the current regular frame.
    vector<Point3f> tracked3DPts;
    
    Mat P1, P2;
    Mat K;
    Mat despL;//feature descriptor for features in left image
    
    Mat tvec, rvec;
    int totalNbFeatures;
    
public:
    Keyframe(Mat _P1,Mat _P2);
    void computeDepth();
    float motionToCurFrame(Frame frame);
    void setNewKeyframe(string imgNameL, string imgNameR);

};






















#endif /* keyframe_hpp */
