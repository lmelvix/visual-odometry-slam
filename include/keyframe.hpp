#ifndef keyframe_hpp
#define keyframe_hpp

#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include "opencv2/calib3d/calib3d.hpp"

#include "frame.hpp"
#include "draw.hpp"

using namespace cv;
using namespace std;

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
    
    Keyframe(Mat _P1,Mat _P2);
    void computeDepth();
    float motionToCurFrame(Frame frame);
    void setNewKeyframe(string imgNameL, string imgNameR);

};
#endif /* keyframe_hpp */
