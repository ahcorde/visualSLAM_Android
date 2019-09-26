// SmallBlurryImage - A small and blurry representation of an image.
// used by the relocaliser.

#ifndef __SMALLBLURRYIMAGE_H
#define __SMALLBLURRYIMAGE_H
#define EIGEN_DONT_ALIGN_STATICALLY True

#include "KeyFrame.h"
#include "ATANCamera.h"
#include "eigen3/Eigen/Dense"


#include "vision/ImageHandler.h"
#include "myWLS.h"
#include "RT.h"

class SmallBlurryImage
{
 public:
  SmallBlurryImage();
  SmallBlurryImage(KeyFrame &kf, double dBlur = 2.5);
  void MakeFromKF(KeyFrame &kf, double dBlur = 2.5);
  void MakeJacs();
  double ZMSSD(SmallBlurryImage &other);
  std::pair<mySE2,double> IteratePosRelToTarget(SmallBlurryImage &other, int nIterations = 10);
  static mySE3 SE3fromSE2(mySE2 se2, ATANCamera camera);

protected:
  cv::Mat mimSmall;
  cv::Mat mimTemplate;
  cv::Mat mimImageJacs;
  bool mbMadeJacs;
  static Eigen::Vector2d mirSize;
};



#endif
