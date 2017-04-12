#ifndef frame_hpp
#define frame_hpp

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include "opencv2/calib3d/calib3d.hpp"

using namespace cv;
using namespace std;

class Frame {

public:
    Mat imgL, imgR;
    vector<KeyPoint> keypointL;
    vector<Point2f> p_keypointL;

    Frame(string filenameL, string filenameR);
    void detectFeatures();
    void getFrame(string filenameL, string filenameR);
};
#endif /* frame_hpp */
