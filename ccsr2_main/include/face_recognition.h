#ifndef _FACE_RECOGNITION_H
#define _FACE_RECOGNITION_H

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"


#define HAAR_CASCADA_FRONTALFACE_DEFAULT "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml"

namespace ccsr2 {

class faceRecognition {

private:

   std::string fn_haar;
   cv::CascadeClassifier haar_cascade;
   std::vector< cv::Rect_<int> > faces;


public:

   faceRecognition();
   void detectFaces(cv::Mat& img);
};

}

#endif
