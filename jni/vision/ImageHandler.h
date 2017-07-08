#ifndef IMAGEHANDLER_H
#define IMAGEHANDLER_H
#define EIGEN_DONT_ALIGN_STATICALLY True

#include <opencv2/core/core.hpp>
#include "/opt/local/include/eigen3/Eigen/Dense"
#include <iostream>
    
  bool in_image_with_border(cv::Mat& img, int px, int py, int border);

  void sample(cv::Mat &im, double x, double y, unsigned char &result);
  void sample(cv::Mat &im, double x, double y, double &result);
  int transform_image(cv::Mat &in, cv::Mat &out, const Eigen::Matrix2d& M, const Eigen::Vector2d& inOrig, const Eigen::Vector2d& outOrig, double defaultValue = 0.0);

  void copy(cv::Mat &in, cv::Mat &out, int w, int h, int col_i, int row_i);

  double FindShiTomasiScoreAtPoint(cv::Mat img, int nsize, int px, int py);

#endif

