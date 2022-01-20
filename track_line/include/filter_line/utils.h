#ifndef _UTILS_H
#define _UTILS_H

#include <opencv2/opencv.hpp>

using namespace cv;

cv::Mat opening_reconstruct(cv::Mat im, int sz);
cv::Mat closing_reconstruct(cv::Mat im, int sz);

// Correct gamma 
cv::Mat correctGamma(cv::Mat img, double gamma);

// C++ (using OpenCV) implementation of some matlab functions
cv::Mat imhmax(cv::Mat im, double thresh);
cv::Mat imhmin(cv::Mat im, double thresh);
cv::Mat imreconstruct(cv::Mat marker, cv::Mat mask);
cv::Mat imregionalmin(cv::Mat img);

void watershedOrg(cv::InputArray _src, cv::InputOutputArray _markers);
void watershedEx(InputArray _src, InputOutputArray _markers);
#endif
