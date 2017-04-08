//
//  optFlwTrack.cpp
//  sceneReconstruct
//
//  Created by 周晓宇 on 3/16/17.
//  Copyright © 2017 xiaoyu. All rights reserved.
//
#include "optFlwTrack.hpp"

using namespace std;

FRAME getFrame(string filename_left, string filename_right,
               Size imageSize){
    
    FRAME frame;
    frame.imgL = imread(filename_left);
    frame.imgR = imread(filename_right);
    
    
//    cv::Ptr<FeatureDetector> _detector = FeatureDetector::create( "GFTT" );
    SurfFeatureDetector _detector(400);
    cv::Ptr<DescriptorExtractor> _descriptor =  DescriptorExtractor::create( "SURF" );
    
    // feature detection
    vector<float> err;
    vector<uchar> status;
    Size winSize = Size(51,51);
    TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,10,0.01);
    Mat homoPts3D, inhomoPts3D;
    clock_t t = clock();
    
    
//    FAST(frame.imgL, frame.keypointL, 5.0);
    _detector.detect(frame.imgL, frame.keypointL);
    KeyPoint::convert(frame.keypointL, frame.p_keypointL);
    
    
    
    t = clock() - t;
//    cout << 1.0*t/CLOCKS_PER_SEC << endl;
    calcOpticalFlowPyrLK(frame.imgL, frame.imgR, frame.p_keypointL,
                         frame.p_keypointR,status, err, winSize,
                         5, termcrit, OPTFLOW_LK_GET_MIN_EIGENVALS,
                         0.001);
    //find min error
//    cout<< *mini << endl;
//    cout<< "position: " << distance(begin(err), mini) << endl;
    vector<Point2f> p_left_trackedPts;
    vector<Point2f> p_right_trackedPts;
//    cout<<"flowed points: "<<status.size() <<endl;
    Point2f pt;
    for(int n = 0; n < status.size(); n++){
        if((matched == status[n])&& (err[n] < 0.25)){
            pt = frame.p_keypointL[n];
            p_left_trackedPts.push_back(pt);
            pt = frame.p_keypointR[n];
            p_right_trackedPts.push_back(pt);
        }
    }
    frame.p_keypointL = p_left_trackedPts;
    frame.p_keypointR = p_right_trackedPts;
    
    KeyPoint::convert(frame.p_keypointL, frame.keypointL);
    KeyPoint::convert(frame.p_keypointR, frame.keypointR);
    
    deleteBadMatch(frame, 1.0, err);

//    cout<<"matched points: " << frame.p_keypointL.size() << endl;
    _descriptor->compute(frame.imgL, frame.keypointL, frame.desp);
    
    drawMatch(frame.imgL, frame.keypointL, frame.keypointR, 1);

    if(waitKey(6) == 27){
        exit;
    }

    return frame;
    
}


void deleteBadMatch(FRAME& frame, float thres, vector<float> err){
    Point2f pt1, pt2;
    KeyPoint kp1, kp2;
    vector<KeyPoint> k_features1, k_features2;
    vector<Point2f> p_features1, p_features2;
    
    int ptsNum = frame.p_keypointL.size();
    
    for(int n = 0; n < ptsNum; n++){
        pt1 = frame.p_keypointL[n];
        pt2 = frame.p_keypointR[n];
        if(fabs(pt1.y-pt2.y) < thres ){
            kp1 = frame.keypointL[n];
            kp2 = frame.keypointR[n];
            k_features1.push_back(kp1);
            k_features2.push_back(kp2);
            p_features1.push_back(pt1);
            p_features2.push_back(pt2);
            
        }
    }
    frame.keypointL = k_features1;
    frame.keypointR = k_features2;
    frame.p_keypointL = p_features1;
    frame.p_keypointR = p_features2;
}

void getDepth(FRAME frame, Mat& inhomo,
              STEREO_RECTIFY_PARAMS srp){

    float uc = srp.P1.at<float>(0,2);
    float vc = srp.P1.at<float>(1,2);
    float f = srp.P1.at<float>(0,0);
    float b = -srp.P2.at<float>(0,3)/f;
    int ptsNum = frame.keypointL.size();
    float d = 0;
    inhomo = Mat(ptsNum, 3, CV_32F);
    
    for(int n = 0; n < ptsNum; n++){
        d = frame.p_keypointL[n].x - frame.p_keypointR[n].x;
        inhomo.at<float>(n,0) = b*(frame.p_keypointR[n].x - uc)/d;
        inhomo.at<float>(n,1) = b*(frame.p_keypointR[n].y - vc)/d;
        inhomo.at<float>(n,2) = b*f/d;
    }
    
}


void estMotionPnP(FRAME frame1, FRAME frame2, STEREO_RECTIFY_PARAMS srp){
    //do feature matching between frame1 and frame2 on the left images,
    //use matched points on the frame1 to do triangulation, to get 3D pts
    //use 3D pts and corresponding features in the frame2 to solve PnP
    
    vector< cv::DMatch > matches;
    cv::FlannBasedMatcher matcher;
//    BFMatcher matcher(NORM_HAMMING);
//    BFMatcher matcher;
    matcher.match( frame1.desp, frame2.desp, matches );
    
    drawMatch(frame2.imgL, frame1.keypointL, frame2.keypointL,matches, 1);
    if(waitKey(5) == 27){
        exit;
    }
    
    cout<<"find total "<<matches.size()<<" matches."<<endl;
    vector< cv::DMatch > goodMatches;
    double minDis = 9999;
    
    for ( int i=0; i<matches.size(); i++ ){
        if ( matches[i].distance < minDis )
            minDis = matches[i].distance;
    }
    
    for ( int i=0; i<matches.size(); i++ ){
        if (matches[i].distance <= 25*minDis)
            goodMatches.push_back( matches[i] );
    }
    cout<<"good matches: "<<goodMatches.size()<<endl;
    // 3d points in frame 1's left image
    //
    vector<cv::Point3f> pts_obj;
    vector< cv::Point2f > pts_img;
    float uc = srp.P1.at<float>(0,2);
    float vc = srp.P1.at<float>(1,2);
    float f = srp.P1.at<float>(0,0);
    float b = -srp.P2.at<float>(0,3)/f;
    float d = 0;
    float minDep=1000, maxDep=0;
    int ptsNum = goodMatches.size();
    Mat inhomo = Mat(ptsNum, 3, CV_32F);

    Point3f pd;
    
    for (int i=0; i<goodMatches.size(); i++){
        pts_img.push_back(Point2f(frame2.keypointL[goodMatches[i].trainIdx].pt));
        
        d = frame1.p_keypointL[i].x - frame1.p_keypointR[i].x;
        pd.x = b*(frame1.p_keypointR[i].x - uc)/d;
        pd.y = b*(frame1.p_keypointR[i].y - vc)/d;
        pd.z = b*f/d;
        
        if(pd.z > maxDep) maxDep = pd.z;
        if(pd.z < minDep) minDep = pd.z;
        
        pts_obj.push_back(pd);
    }
    
    cout <<"closest depth: " << minDep<< endl;
    cout <<"far depth: " << maxDep << endl;
    
    Mat K;
    srp.P1.colRange(0, 3).copyTo(K);

    Mat rvec, tvec, inliers;
    solvePnPRansac( pts_obj, pts_img, K, Mat(),
                   rvec, tvec, false, 1000, 5.0, 20, inliers,
                   CV_EPNP);
    
    Mat tvectp = tvec.t();
    
    cout << tvec <<endl;
    cout <<" ,  inlier numbers: " << inliers.rows << endl;

    vector<KeyPoint> matched1, matched2;
    KeyPoint pt;
    int idx;
    for (int n = 0; n < inliers.rows; n++){
        idx = inliers.at<int>(n,0);
        pt = frame1.keypointL[goodMatches[idx].queryIdx];
        matched1.push_back(pt);
        pt = frame2.keypointL[goodMatches[idx].trainIdx];
        matched2.push_back(pt);
    }
    drawMatch(frame2.imgL, matched1, matched2, 1);
    if(waitKey(5) == 27){
        exit;
    }
    
    
    
}


void estMotionPnPOpt(FRAME frame1, FRAME frame2, STEREO_RECTIFY_PARAMS srp){
    
    vector<float> err;
    vector<uchar> status;
    Size winSize = Size(51,51);
    TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,10,0.01);
    
    calcOpticalFlowPyrLK(frame1.imgL, frame2.imgL, frame1.p_keypointL,
                         frame2.p_keypointL, status, err, winSize,
                         5, termcrit, OPTFLOW_LK_GET_MIN_EIGENVALS, 0.001);
    //get correctly tracked points
    vector<Point2f> p_prev_trackedPtsL;
    vector<Point2f> p_prev_trackedPtsR;
    vector<Point2f> p_curr_trackedPts;
//    cout<<"flowed points: "<<status.size() <<endl;
    Point2f pt;
    for(int n = 0; n < status.size(); n++){
        if((matched == status[n])&& (err[n] < 0.25)){
            pt = frame1.p_keypointL[n];
            p_prev_trackedPtsL.push_back(pt);
            pt = frame1.p_keypointR[n];
            p_prev_trackedPtsR.push_back(pt);
            pt = frame2.p_keypointL[n];
            p_curr_trackedPts.push_back(pt);
        }
    }
    if(0 == p_curr_trackedPts.size()){
        return;
    }
    //obtain 3D points
    vector<cv::Point3f> pts_obj;
    vector< cv::Point2f > pts_img = p_curr_trackedPts;
    float uc = srp.P1.at<float>(0,2);
    float vc = srp.P1.at<float>(1,2);
    float f = srp.P1.at<float>(0,0);
    float b = -srp.P2.at<float>(0,3)/f;
    float d = 0;
    float minDep=1000, maxDep=0;
    int ptsNum = p_curr_trackedPts.size();
    Mat inhomo = Mat(ptsNum, 3, CV_32F);
    
    Point3f pd;
    
    for (int i=0; i<p_curr_trackedPts.size(); i++){
        d = p_prev_trackedPtsL[i].x - p_prev_trackedPtsR[i].x;
        pd.x = b*(p_prev_trackedPtsL[i].x - uc)/d;
        pd.y = b*(p_prev_trackedPtsL[i].y - vc)/d;
        pd.z = b*f/d;
        
        if(pd.z > maxDep) maxDep = pd.z;
        if(pd.z < minDep) minDep = pd.z;
        
        pts_obj.push_back(pd);
    }
    
//    cout <<"closest depth: " << minDep<< endl;
//    cout <<"far depth: " << maxDep << endl;
    
    //run PnP
    Mat K;
    srp.P1.colRange(0, 3).copyTo(K);
    
    Mat rvec, tvec, inliers;
    solvePnPRansac( pts_obj, pts_img, K, Mat(),
                   rvec, tvec, false, 100, 2.0, 10, inliers);
    // can be modified by using RANSAC for linear, and then use LM to
    // optimize

    cout << tvec.t();
    cout << "inlier numbers: " << inliers.rows << endl;
    
    vector<Point2f> matched1, matched2;
//    KeyPoint kpt;
    int idx;
    for (int n = 0; n < inliers.rows; n++){
        idx = inliers.at<int>(n,0);
        pt = p_prev_trackedPtsL[idx];
        matched1.push_back(pt);
        pt = p_curr_trackedPts[idx];
        matched2.push_back(pt);
    }
    vector<KeyPoint> kmatched1, kmatched2;
    KeyPoint::convert(matched1, kmatched1);
    KeyPoint::convert(matched2, kmatched2);
    
    drawMatch(frame2.imgL, kmatched1, kmatched2, 1);
    if(waitKey(5) == 27){
        exit;
    }
    
}


void estMotionPnPOptFlw(Keyframe keyframe, Frame frame, Mat P2){
    vector<float> err;
    vector<uchar> status;
    Size winSize = Size(51,51);
    TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,10,0.01);
    
    calcOpticalFlowPyrLK(keyframe.imgL, frame.imgL, keyframe.p_keypointL,
                         frame.p_keypointL, status, err, winSize,
                         5, termcrit, OPTFLOW_LK_GET_MIN_EIGENVALS, 0.001);
    
    //get successfully tracked points
    vector<Point2f> p_prev_trackedPts;
    vector<Point2f> pts_img;
    vector<Point3f> pts_obj;
    
    Point2f pt_img;
    Point3f pt_obj;
    for(int n = 0; n < status.size(); n++){
        if((matched == status[n])&& (err[n] < 0.25)){
            pt_img = keyframe.p_keypointL[n];
            p_prev_trackedPts.push_back(pt_img);
            
            pt_img = frame.p_keypointL[n];
            pts_img.push_back(pt_img);
            
            pt_obj = keyframe.scenePts[n];
            pts_obj.push_back(pt_obj);
        }
    }
    if(0 == pts_img.size()){
        return;
    }
    //solve PnP
    Mat K;
    P2.colRange(0, 3).copyTo(K);
    
    Mat rvec, tvec, inliers;
    solvePnPRansac( pts_obj, pts_img, K, Mat(),
                   rvec, tvec, false, 100, 2.0, 10, inliers);
    // can be modified by using RANSAC for linear, and then use LM to
    // optimize
    
    cout << tvec.t();
    cout << "inlier numbers: " << inliers.rows << endl;
    
//    vector<Point2f> matched1, matched2;
//    //    KeyPoint kpt;
//    int idx;
//    for (int n = 0; n < inliers.rows; n++){
//        idx = inliers.at<int>(n,0);
//        pt = p_prev_trackedPtsL[idx];
//        matched1.push_back(pt);
//        pt = p_curr_trackedPts[idx];
//        matched2.push_back(pt);
//    }
//    vector<KeyPoint> kmatched1, kmatched2;
//    KeyPoint::convert(matched1, kmatched1);
//    KeyPoint::convert(matched2, kmatched2);
//    
//    drawMatch(frame2.imgL, kmatched1, kmatched2, 1);
//    if(waitKey(5) == 27){
//        exit;
//    }
//
    
}












double normofTransform( cv::Mat rvec, cv::Mat tvec ){
    return fabs(MIN(norm(rvec), 2*M_PI-norm(rvec)))+ fabs(norm(tvec));
}







void convertMattoPtsCloud(Mat inhomoPts3D, PointCloud<PointXYZ>& ptCloud){
    
    int r = inhomoPts3D.rows;
    int c = inhomoPts3D.cols;
    bool rowRep = TRUE;
    int ptsNum;
    
    if(3 == r){
        rowRep = FALSE;
        ptsNum = c;
    }
    else if(3 == c){
        ptsNum = r;
    }
    ptCloud.width = ptsNum; // specify
    ptCloud.height = 1;
    ptCloud.is_dense = false;
    ptCloud.points.resize(ptCloud.width * ptCloud.height);
    
    if(TRUE == rowRep){
        for(int n = 0; n < ptsNum; n++){
            ptCloud.points[n].x = inhomoPts3D.at<float>(n, 0);
            ptCloud.points[n].y = -inhomoPts3D.at<float>(n, 1);
            ptCloud.points[n].z = inhomoPts3D.at<float>(n, 2);
        }
    }
    else{
        for(int n = 0; n < ptsNum; n++){
            //            cout << inhomoPts3D.at<float>(0,n);
            ptCloud.points[n].x = inhomoPts3D.at<float>(0, n);
            ptCloud.points[n].y = -inhomoPts3D.at<float>(1, n);
            ptCloud.points[n].z = inhomoPts3D.at<float>(2, n);
        }
    }
}



void convert3DHomoToInhomo(Mat homoPts3D, Mat& inhomoPts3D){
    //convert 3d homogeneous points to inhomogeneous point.
    //each column represents a point.
    Mat T, tempDst, tempSrc;
    int ptsNum = homoPts3D.cols;
    inhomoPts3D = Mat::zeros(3, ptsNum, CV_32F);
    homoPts3D.row(3).copyTo(T);
    
    
    for(int n = 0; n < 3; n++){
        homoPts3D.row(n).copyTo(tempSrc);
        divide(tempSrc, T, tempDst);
        tempDst.copyTo(inhomoPts3D.row(n));
    }
}



Mat getCameraProjMat(CAMERA_INTRINSIC K, CAMERA_EXTRINSIC stereo){
    Mat K_cam = Mat::zeros(3,3,CV_32F);
    Mat projMat;// = Mat::zeros(3,4,CV_32F);
    Mat camMat;
    K_cam.at<float>(0,0) = K.fx;
    K_cam.at<float>(1,1) = K.fy;
    K_cam.at<float>(0,2) = K.cx;
    K_cam.at<float>(1,2) = K.cy;
    K_cam.at<float>(2,2) = 1.0;
    
    hconcat(stereo.R, stereo.t, projMat);
    
    gemm(K_cam, projMat, 1, NULL, 0, camMat);
    cout << camMat << endl;
    return camMat;
}

Mat getCameraMat(CAMERA_INTRINSIC K){
    Mat K_cam = Mat::zeros(3,3,CV_32F);
    K_cam.at<float>(0,0) = K.fx;
    K_cam.at<float>(1,1) = K.fy;
    K_cam.at<float>(0,2) = K.cx;
    K_cam.at<float>(1,2) = K.cy;
    K_cam.at<float>(2,2) = 1.0;
    
    return K_cam;
}






