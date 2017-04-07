//
//  main.cpp
//  sceneReconstruct
//
//  Created by 周晓宇 on 3/16/17.
//  Copyright © 2017 xiaoyu. All rights reserved.
//
//  use strict standard to track features and camera, and decide should I add a new keyframe by according to baseline distance and number of tracked features. then use a more tolerant standard to judge should I do a triangulation.
//
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

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/icp.h>
//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/surface/gp3.h>


#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>

#include <boost/thread.hpp>

#include "optFlwTrack.hpp"
#include "keyframe.hpp"
#include "frame.hpp"
#include "draw.hpp"
#include "basic.hpp"

using namespace cv;
using namespace std;
using namespace pcl;

//RNG rng(12345);
//Keyframe getNewKeyframe(string imgNameL, string imgNameR,
//                        Mat _P1, Mat _P2);


int main(int argc, const char * argv[]) {
    string leftImgPath ="/Users/XiaoyuZhou/Google Drive/UCSD_2017SPRING/CSE252C/data/image_0";
    string rightImgPath="/Users/XiaoyuZhou/Google Drive/UCSD_2017SPRING/CSE252C/data/image_1";
    string seqPath = "/Users/XiaoyuZhou/Google Drive/UCSD_2017SPRING/CSE252C/data/stereo252.txt";
    vector<string> leftImgName;
    vector<string> rightImgName;
    
    LoadImages(leftImgPath, rightImgPath, seqPath,
               leftImgName, rightImgName);
    
    
    Mat test1 = imread(leftImgName[0]);
    float row = (float)test1.rows;
    float col = (float)test1.cols;
    Size imageSize = Size(col, row);
  
    STEREO_RECTIFY_PARAMS srp;
    srp.P1 = (Mat_<float>(3,4) <<
              707.0912,    0.0, 601.8873, 0.0,
                0.0 , 707.0912, 183.1104, 0.0,
                0.0,     0.0  ,   1.0,    0.0);
    srp.P2 = (Mat_<float>(3,4) <<
              707.0912,    0.0, 601.8873, -379.8145,
                0.0 , 707.0912, 183.1104, 0.0,
                0.0,     0.0  ,   1.0,    0.0);
    
    
    //====prepare rectification parameters========================================
    clock_t t;

    // parameter setup
    vector<Keyframe> keyframe;
    

//    Ptr<FeatureDetector>_detector = FeatureDetector::create("SURF");
    
    //triangulation
    Mat homoPts3D, inhomoPts3D; //used to store homogeneous 3D points, each column
    Mat inhomo;                 //represents a point
//    Mat imgCur, imgR;
    PointCloud<PointXYZ> oldPtCloud, newPtCloud;

    
    //visulization
    PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
    *cloud = oldPtCloud;
    boost::shared_ptr<visualization::PCLVisualizer> cloudView (new visualization::PCLVisualizer ("3D Viewer"));
    cloudView->setBackgroundColor(0, 0, 0);
    cloudView->addCoordinateSystem();
    cloudView->addPointCloud(cloud);

    //ICP
    IterativeClosestPoint<PointXYZ, PointXYZ> icp;
    PointCloud<PointXYZ>::Ptr icpOld (new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr icpNew(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ> finalPtCloud;
//=============== initialize system ============================================

//    FRAME frame = getFrame(leftImgName[0], rightImgName[0],imageSize);
//    getDepth(frame, inhomo, srp);
//    
//    PointCloud<PointXYZ> ptsCloud;
//    convertMattoPtsCloud(inhomo, ptsCloud);
//    *icpOld = ptsCloud;
//    cloudView->updatePointCloud(icpOld);
//    
//    if(waitKey(5) == 27){
//        exit;
//    }
//    testKeyframe = frame;
    int count = 1;
    float featureRatio = 0;
    
    Keyframe lastKeyframe(srp.P1, srp.P2);
    lastKeyframe.setNewKeyframe(leftImgName[0], rightImgName[0]);
    keyframe.push_back(lastKeyframe);
    Frame curFrame(leftImgName[1], rightImgName[1]);
//
    featureRatio = keyframe.back().motionToCurFrame(curFrame);
//    while(!cloudView->wasStopped()){
//        cloudView->spinOnce(100);
//        boost::this_thread::sleep(boost::posix_time::microseconds (10));
//    }
    
    
//============== main loop ============================================================
    while(1){
        
        curFrame.getFrame(leftImgName[count], rightImgName[count]);
        featureRatio = keyframe.back().motionToCurFrame(curFrame);
        
//        double move = normofTransform(keyframe.back().rvec, keyframe.back().tvec);
        
        cout << move << endl;
        
        if(featureRatio < 0.5){
//            cout << "new keyframe added" << endl;
            lastKeyframe.setNewKeyframe(leftImgName[count], rightImgName[count]);
            keyframe.push_back(lastKeyframe);
            
            
        }
        

    count++;
    }
    
//=========== main loop ends ===========================================================
//    *cloud = finalPtCloud;
//    cloudView->updatePointCloud(cloud);
//
//    
//    cout << "final point clouds overview: " << finalPtCloud << endl;
//    
//    
//    while(!cloudView->wasStopped()){
//        cloudView->spinOnce(100);
//        boost::this_thread::sleep(boost::posix_time::microseconds (10));
//    }
//    
    
    waitKey(0);
//    io::savePCDFileASCII("/Users/XiaoyuZhou/Desktop/CV/Xcode/sceneReconstruct/pt.pcd", finalPtCloud);
//            cout << "Saved " << cloud_filtered.points.size() << " data points to test_pcd.pcd." << std::endl;
    return 0;
}
//Keyframe getNewKeyframe(string imgNameL, string imgNameR,
//                        Mat _P1, Mat _P2){
//    Keyframe newkeyframe;
//    return newkeyframe(imgNameL, imgNameR, _P1, _P2);
//
//    
//}
