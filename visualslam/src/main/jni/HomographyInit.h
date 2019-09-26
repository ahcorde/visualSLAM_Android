// Declares the HomographyInit class and a few helper functions. 
//
// This class is used by MapMaker to bootstrap the map, and implements
// the homography decomposition of Faugeras and Lustman's 1988 tech
// report.
//
// Implementation according to Faugeras and Lustman

#ifndef __HOMOGRAPHY_INIT_H
#define __HOMOGRAPHY_INIT_H
#define EIGEN_DONT_ALIGN_STATICALLY True

#include <vector>
#include "eigen3/Eigen/Dense"


#include "myWLS.h"
#include "RT.h"
#include "MEstimator.h"
#include <iostream>

// Homography matches are 2D-2D matches in a stereo pair, unprojected
// to the Z=1 plane.
struct HomographyMatch {
  // To be filled in by MapMaker:
  Eigen::Vector2d v2CamPlaneFirst;
  Eigen::Vector2d v2CamPlaneSecond;
  Eigen::MatrixXd m2PixelProjectionJac;
};

// Storage for each homography decomposition
struct HomographyDecomposition {
  Eigen::Vector3d v3Tp;
  Eigen::Matrix3d m3Rp;
  double d;

  Eigen::Vector3d v3n;

  // The resolved composition..
  mySE3 se3SecondFromFirst;
  int nScore;
};

class HomographyInit {
public:

  bool Compute(std::vector<HomographyMatch> vMatches, double dMaxPixelError, mySE3 &se3SecondCameraPose);

protected:

  Eigen::Matrix3d HomographyFromMatches(std::vector<HomographyMatch> vMatches);
  void BestHomographyFromMatches_MLESAC();
  void DecomposeHomography();
  void ChooseBestDecomposition();
  void RefineHomographyWithInliers(); 
  bool IsHomographyInlier(Eigen::Matrix3d &m3Homography, HomographyMatch match);
  double MLESACScore(Eigen::Matrix3d &m3Homography, HomographyMatch match);
  
  double mdMaxPixelErrorSquared;
  Eigen::Matrix3d mm3BestHomography;
  std::vector<HomographyMatch> mvMatches;
  std::vector<HomographyMatch> mvHomographyInliers;
  std::vector<HomographyDecomposition> mvDecompositions;
};

#endif
