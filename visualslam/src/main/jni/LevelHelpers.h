// LevelHelpers.h - a few handy tools to ease using levels.
// The important thing is the XXXPos functions, which convert
// image positions from one level to another. Use these whenever
// transforming positions to ensure consistent operation!!

#ifndef __LEVEL_HELPERS_H
#define __LEVEL_HELPERS_H
#define EIGEN_DONT_ALIGN_STATICALLY True

#include "eigen3/Eigen/Dense"



// Set of global colours useful for drawing stuff:
extern Eigen::Vector3d gavLevelColors[];
// (These are filled in in KeyFrame.cc)

// What is the scale of a level?
inline int LevelScale(int nLevel) {
  return 1 << nLevel;
}

// 1-D transform to level zero:
inline double LevelZeroPos(double dLevelPos, int nLevel) {
  return (dLevelPos + 0.5) * LevelScale(nLevel) - 0.5;
}

// 2-D transforms to level zero:
inline Eigen::Vector2d LevelZeroPos(Eigen::Vector2d &v2LevelPos, int nLevel) {
  Eigen::Vector2d v2Ans;
  v2Ans(0) = LevelZeroPos((double)v2LevelPos[0], nLevel);
  v2Ans(1) = LevelZeroPos((double)v2LevelPos(1), nLevel);
  return v2Ans;
}

// 1-D transform from level zero to level N:
inline double LevelNPos(double dRootPos, int nLevel) {
  return (dRootPos + 0.5) / LevelScale(nLevel) - 0.5;
}

// 2-D transform from level zero to level N:
inline Eigen::Vector2d LevelNPos(Eigen::Vector2d v2RootPos, int nLevel) {
  Eigen::Vector2d v2Ans;
  v2Ans(0) = LevelNPos((double)v2RootPos(0), nLevel);
  v2Ans(1) = LevelNPos((double)v2RootPos(1), nLevel);
  return v2Ans;
}

#endif
