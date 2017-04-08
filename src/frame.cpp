//
//  frame.cpp
//  sceneReconstruct
//
//  Created by 周晓宇 on 4/3/17.
//  Copyright © 2017 xiaoyu. All rights reserved.
//

#include "frame.hpp"

using namespace std;

Frame::Frame(string filenameL, string filenameR){
    imgL = imread(filenameL);
    imgR = imread(filenameR);
    cvtColor( imgL  , imgL, CV_BGR2GRAY );
    cvtColor( imgR  , imgR, CV_BGR2GRAY );
    GaussianBlur(imgL, imgL, Size(7,7), 0.1);
    GaussianBlur(imgR, imgR, Size(7,7), 0.1);
    if(!imgL.data || ! imgR.data){
        std::cout << "image does not exist..." << std::endl;
        //exit;
    }
}

void Frame::detectFeatures(){
    SurfFeatureDetector surf;
    surf.detect(imgL, keypointL);
    KeyPoint::convert(keypointL, p_keypointL);
}

void Frame::getFrame(string filenameL, string filenameR){
    imgL = imread(filenameL);
    imgR = imread(filenameR);
    
    if(!imgL.data || ! imgR.data){
        std::cout << "image does not exist..." << std::endl;
        //exit;
    }
    cvtColor( imgL  , imgL, CV_BGR2GRAY );
    cvtColor( imgR  , imgR, CV_BGR2GRAY );
}
