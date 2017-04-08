/*
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

#include "optFlwTrack.hpp"
#include "keyframe.hpp"
#include "frame.hpp"
#include "draw.hpp"
#include "basic.hpp"

using namespace cv;
using namespace std;
*/

#include "main.hpp"

int main(int argc, const char * argv[]) {

    string leftImgPath = LEFT_IMAGE;
    string rightImgPath = RIGHT_IMAGE;
    string seqPath = STEREO;
    vector<string> leftImgName;
    vector<string> rightImgName;
    
    cout<<"Entering the dungeon !"; 

    LoadImages(leftImgPath, rightImgPath, seqPath, leftImgName, rightImgName);
    
    Mat test1 = imread(leftImgName[0]);
    STEREO_RECTIFY_PARAMS srp;
    srp.P1 = (
              Mat_<float>(3,4) << 707.0912, 0.0,      601.8873, 0.0,
                                  0.0 ,     707.0912, 183.1104, 0.0,
                                  0.0,      0.0  ,    1.0,      0.0
             );
    srp.P2 = (
              Mat_<float>(3,4) << 707.0912, 0.0,      601.8873, -379.8145,
                                  0.0 ,     707.0912, 183.1104,  0.0,
                                  0.0,      0.0  ,    1.0,       0.0
             );
    
    // parameter setup
    vector<Keyframe> keyframe;
    
    //triangulation
    Mat homoPts3D, inhomoPts3D; //used to store homogeneous 3D points, each column
    Mat inhomo;                 //represents a point
    int count = 1;
    float featureRatio = 0;
    
    Keyframe lastKeyframe(srp.P1, srp.P2);
    lastKeyframe.setNewKeyframe(leftImgName[0], rightImgName[0]);
    keyframe.push_back(lastKeyframe);
    Frame curFrame(leftImgName[1], rightImgName[1]);
    featureRatio = keyframe.back().motionToCurFrame(curFrame);

    while(1){
        curFrame.getFrame(leftImgName[count], rightImgName[count]);
        featureRatio = keyframe.back().motionToCurFrame(curFrame);
        
        if(featureRatio < 0.5){
            lastKeyframe.setNewKeyframe(leftImgName[count], rightImgName[count]);
            keyframe.push_back(lastKeyframe);
        }
    count++;
    }
   
    waitKey(0);

    return 0;
}
