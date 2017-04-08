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

 #define LEFT_IMAGE "../data/dataset/sequence/08/image_0/"
 #define RIGHT_IMAGE "../data/dataset/sequence/08/image_1/"
 #define STEREO "stereo252.txt"

 using namespace cv;
 using namespace std;

