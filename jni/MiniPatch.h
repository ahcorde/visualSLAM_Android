// Declares MiniPatch class
// 
// This is a simple pixel-patch class, used for tracking small patches
// it's used by the tracker for building the initial map

#ifndef __MINI_PATCH_H
#define __MINI_PATCH_H
#define EIGEN_DONT_ALIGN_STATICALLY True

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>
#include "/Users/ahcorde/Downloads/eigen/Eigen/Dense"
#include "vision/ImageHandler.h"

struct MiniPatch
{
  void SampleFromImage(Eigen::Vector2d irPos, cv::Mat &im);  // Copy pixels out of source image
  bool FindPatch(Eigen::Vector2d &irPos, cv::Mat &im, int nRange, std::vector<Eigen::Vector2d> &vCorners, std::vector<int> *pvRowLUT = NULL); // Find patch in a new image
  
  inline int SSDAtPoint(cv::Mat &im, int icol, int irow); // Score function
  static int mnHalfPatchSize;     // How big is the patch?
  static int mnRange;             // How far to search? 
  static int mnMaxSSD;            // Max SSD for matches?
  cv::Mat mimOrigPatch;  // Original pixels
};

#endif
