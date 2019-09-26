#ifndef CVFAST_H
#define CVFAST_H
#define EIGEN_DONT_ALIGN_STATICALLY True

#include <opencv/cvaux.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdlib.h>
#include <vector>
#include "eigen3/Eigen/Dense"


CVAPI(void)  cvCornerFast(cv::Mat &src, int threshold, int N,	int nonmax_suppression, int* ret_number_of_corners,	CvPoint** ret_corners, int ** scores);
CVAPI(void)  cvCornerFast_10(cv::Mat &src, std::vector<Eigen::Vector2d> &corners, int threshold);

CVAPI(void) fast_nonmax(cv::Mat &src, std::vector<Eigen::Vector2d> &corners, int barrier, std::vector<Eigen::Vector2d> &max_corners);

#endif
