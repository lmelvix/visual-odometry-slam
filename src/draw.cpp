#include "draw.hpp"

#define matched  1
using namespace cv;

void drawFeature(Mat img, vector<KeyPoint> features, char* windowName){
    
    int ptsNum = (int)features.size();
    vector<Point2f> p_features;
    p_features.resize(ptsNum);
    
    Mat copy;
    copy = img.clone();
    KeyPoint::convert(features, p_features);
    for( int i = 0; i < p_features.size(); i++ ){
        circle( copy, p_features[i], 2, Scalar(0,250,0), 1, 8, 0 );
    }
    imshow(windowName, copy);
    
}
