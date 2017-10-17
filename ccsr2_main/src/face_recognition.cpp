//#include "opencv2/core/core.hpp"
//#include "opencv2/contrib/contrib.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"

#include <iostream>
#include <fstream>
#include <sstream>

#include "face_recognition.h"




ccsr2::faceRecognition::faceRecognition() {
   fn_haar = HAAR_CASCADA_FRONTALFACE_DEFAULT;
   haar_cascade.load(fn_haar);
}

void ccsr2::faceRecognition::detectFaces(cv::Mat& img) {

   cv::Mat original = img.clone();
   // Convert the current frame to grayscale:
   cv::Mat gray;
   cv::cvtColor(original, gray, CV_BGR2GRAY);
   // Find the faces in the frame:
   haar_cascade.detectMultiScale(gray, faces);
   // At this point you have the position of the faces in
   // faces. Now we'll get the faces, make a prediction and
   // annotate it in the video. Cool or what?
   for(int i = 0; i < faces.size(); i++) {
      // Process face by face:
      cv::Rect face_i = faces[i];
      cv::rectangle(img, face_i, CV_RGB(0, 255,0), 1);
   }
}
