//
//  keyframe.cpp
//  sceneReconstruct
//
//  Created by 周晓宇 on 4/3/17.
//  Copyright © 2017 xiaoyu. All rights reserved.
//

#include "keyframe.hpp"

Keyframe::Keyframe(Mat _P1, Mat _P2){
    P1 = _P1;
    P2 = _P2;
    P2.colRange(0, 3).copyTo(K);
}
void Keyframe::computeDepth(){
    float uc = P2.at<float>(0,2);
    float vc = P2.at<float>(1,2);
    float f = P2.at<float>(0,0);
    float b = -P2.at<float>(0,3)/f;
    float d = 0;
    //from paper: Vision meets Robotics: The KITTI Dataset
    //the format of P2 is:
    //  f_u,    0,    c_u,    -f_u*b_x
    //   0 ,  f_v,    c_v,        0
    //   0 ,    0,      1,        0
    // the origin is at the right camera, which is unusual
    Point3f pd;
    for( int n = 0; n<keypointL.size(); n++){
        d = p_keypointL[n].x - p_keypointR[n].x;
        pd.x = b*(p_keypointL[n].x - uc)/d;
        pd.y = b*(p_keypointL[n].y - vc)/d;
        pd.z = b*f/d;
        
        scenePts.push_back(pd);
    }
}

float Keyframe::motionToCurFrame(Frame frame){
    vector<Point2f> store_trackedPts = trackedPts;
    vector<Point3f> store_tracked3DPts = tracked3DPts;
    vector<int> store_trackedPtsIdx = trackedPtsIdx;
    Mat store_tvec = tvec;
    Mat store_rvec = rvec;
    
    vector<float> err;
    vector<uchar> status;
    Size winSize = Size(21,41);
    TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,50,0.005);
    
    calcOpticalFlowPyrLK(imgL, frame.imgL, trackedPts,
                         frame.p_keypointL, status, err, winSize,
                         3, termcrit, OPTFLOW_LK_GET_MIN_EIGENVALS, 0.1);

    //get successfully tracked points
    vector<Point2f> p_trackedPts;
    vector<Point2f> pts_img;//store feature in regular frame
    vector<Point3f> pts_obj;//store 3D points in keyframe
    vector<int> tempIdx;
    
    Point2f pt_img;
    Point3f pt_obj;
    for(int n = 0; n < status.size(); n++){
        if((1 == status[n])&& (err[n] < 0.5)){
            //tracked points in keyframe, for drawing
            pt_img = p_keypointL[trackedPtsIdx[n]];
            p_trackedPts.push_back(pt_img);
            
            //tracked points in current regular frame.
            pt_img = frame.p_keypointL[n];
            pts_img.push_back(pt_img);
            
            //tracked 3D points in original, for updating
            pt_obj = scenePts[trackedPtsIdx[n]];
            pts_obj.push_back(pt_obj);
            
            tempIdx.push_back(trackedPtsIdx[n]);
        }
    }
    if(0 == pts_img.size()){
        cout << "not enough points!!!!   "<< endl;
        return 0;
    }
    trackedPts = pts_img;
    tracked3DPts = pts_obj;
    
    trackedPtsIdx = tempIdx;
    
//    cout << "remained feature ratio: " << 1.0*trackedPtsIdx.size()/totalNbFeatures << endl;
    
    vector<KeyPoint> kmatched1, kmatched2;
    KeyPoint::convert(p_trackedPts, kmatched1);
    KeyPoint::convert(pts_img, kmatched2);
    
    Mat copy;
    cvtColor(frame.imgL, copy, CV_GRAY2RGB);
//    drawMatch(copy, kmatched1, kmatched2, 1);
    drawFeature(copy, kmatched2, "track");
    if(waitKey(5) == 27){
        exit;
    }
    
    //solve PnP
    
    Mat inliers;
    solvePnPRansac(pts_obj, pts_img, K, Mat(),
                   rvec, tvec, false, 100, 5.0, 10, inliers);
    
//    if(fabs(MIN(norm(rvec), 2*M_PI-norm(rvec)))+ fabs(norm(tvec)) > 10){
//        cout << "bad frame, discarded!" << endl;
//        trackedPts = store_trackedPts;
//        tracked3DPts = store_tracked3DPts;
//        trackedPtsIdx = store_trackedPtsIdx;
//        rvec = store_rvec;
//        tvec = store_tvec;
//    }
    
//    cout << tvec.t();
//    cout << "inlier numbers: " << inliers.rows << endl;

    return 1.0*trackedPtsIdx.size() / totalNbFeatures;
}
/*void Keyframe::motionToCurFrame(Frame frame){
    vector<float> err;
    vector<uchar> status;
    Size winSize = Size(51,51);
    TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,10,0.01);
    
    calcOpticalFlowPyrLK(imgL, frame.imgL, p_keypointL,
                         frame.p_keypointL, status, err, winSize,
                         5, termcrit, OPTFLOW_LK_GET_MIN_EIGENVALS, 0.001);
    
    //get successfully tracked points
    vector<Point2f> p_trackedPts;
    vector<Point2f> pts_img;//store feature in regular frame
    vector<Point3f> pts_obj;//store 3D points in keyframe
    
    Point2f pt_img;
    Point3f pt_obj;
    for(int n = 0; n < status.size(); n++){
        if((1 == status[n])&& (err[n] < 0.25)){
            //tracked points in keyframe, for updating
            pt_img = p_keypointL[n];
            p_trackedPts.push_back(pt_img);
            
            //tracked points in current regular frame.
            pt_img = frame.p_keypointL[n];
            pts_img.push_back(pt_img);
            
            //tracked 3D points in keyframe, for updating
            pt_obj = scenePts[n];
            pts_obj.push_back(pt_obj);
        }
    }
    if(0 == pts_img.size()){
        cout << "not enough points!!!!   "<< endl;
        return;
    }
    //solve PnP
    
    Mat inliers;
    solvePnPRansac(pts_obj, pts_img, K, Mat(),
                   rvec, tvec, false, 100, 5.0, 10, inliers);
    // can be modified by using RANSAC for linear, and then use LM to
    // optimize
    
    cout << tvec.t();
    cout << "inlier numbers: " << inliers.rows << endl;
    
    
    //visuliziation/ select tracked points
    vector<Point2f> matched1, matched2;
    int idx;
    for (int n = 0; n < inliers.rows; n++){
        idx = inliers.at<int>(n,0);
        
        pt_img = p_trackedPts[idx];
        matched1.push_back(pt_img);//1 for keyframe
        
        pt_img = pts_img[idx];
        matched2.push_back(pt_img);//2 for regular frame
        
    }
    vector<KeyPoint> kmatched1, kmatched2;
    KeyPoint::convert(matched1, kmatched1);
    KeyPoint::convert(matched2, kmatched2);
    
    drawMatch(frame.imgL, kmatched1, kmatched2, 1);
    if(waitKey(5) == 27){
        exit;
    }
    
}*/

void Keyframe::setNewKeyframe(string imgNameL, string imgNameR){
    imgL = imread(imgNameL);
    imgR = imread(imgNameR);

    if(!imgL.data || ! imgR.data){
        cout << "image does not exist..." << endl;
        exit;
    }
    
    Ptr<DescriptorExtractor> _descriptor =  DescriptorExtractor::create( "SURF" );
    
    
    cvtColor( imgL  , imgL, CV_BGR2GRAY );
    cvtColor( imgR  , imgR, CV_BGR2GRAY );
    
    GaussianBlur(imgL, imgL, Size(7,7), 0.1);
    GaussianBlur(imgR, imgR, Size(7,7), 0.1);
    
    
    goodFeaturesToTrack(imgL, p_keypointL, 400, 0.01, 10,Mat(),15, true,0.01);
//    cornerHarris(imgL, p_keypointL, 10, 7, 0.04);
//    OrbFeatureDetector _detect(200);
    
//        _detect.detect(imgL, keypointL);
//        KeyPoint::convert(keypointL, p_keypointL);
//    KeyPoint::convert(p_keypointL, keypointL);
    
    vector<float> err;
    vector<uchar> status;
    Size winSize = Size(61,61);
    TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,50,0.01);
    
    calcOpticalFlowPyrLK(imgL, imgR, p_keypointL,p_keypointR,
                         status, err, winSize,
                         6, termcrit, OPTFLOW_LK_GET_MIN_EIGENVALS, 0.001);
    //========= select good matches================
    vector<Point2f> p_left_trackedPts;
    vector<Point2f> p_right_trackedPts;
    
    Point2f pt1, pt2;
    KeyPoint kp1, kp2;
    vector<KeyPoint> k_features1, k_features2;
    vector<Point2f> p_features1, p_features2;
    
    for(int n = 0; n < status.size(); n++){
        if((1 == status[n]) && (err[n] < 0.25)){
            pt1 = p_keypointL[n];
            p_left_trackedPts.push_back(pt1);
            pt1 = p_keypointR[n];
            p_right_trackedPts.push_back(pt1);
        }
    }
    p_keypointL = p_left_trackedPts;
    p_keypointR = p_right_trackedPts;
    
    KeyPoint::convert(p_keypointL, keypointL);
    KeyPoint::convert(p_keypointR, keypointR);
    
    //=====delete bad matches========================
    //if the value of y-axis between two corresponding feature points
    //is larger than a threshold, then it is a bad match
    for(int n = 0; n < p_keypointL.size(); n++){
        pt1 = p_keypointL[n];
        pt2 = p_keypointR[n];
        if(fabs(pt1.y-pt2.y) < 2.0){
            kp1 = keypointL[n];
            kp2 = keypointR[n];
            k_features1.push_back(kp1);
            k_features2.push_back(kp2);
            p_features1.push_back(pt1);
            p_features2.push_back(pt2);
        }
    }
    keypointL = k_features1;
    keypointR = k_features2;
    p_keypointL = p_features1;
    p_keypointR = p_features2;
    
    trackedPts = p_keypointL;
    //====get 3D points===========================
    scenePts.clear();
    computeDepth();
    tracked3DPts = scenePts;
    for(int i = 0; i < trackedPts.size(); i++){
        trackedPtsIdx.push_back(i);
    }
    totalNbFeatures = scenePts.size();
}


void convertMatToVec(Mat input, vector<Point2f>output){
    
    
    
}

















