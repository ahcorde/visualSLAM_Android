// SmallBlurryImage-based relocaliser
// 
// Each KF stores a small, blurred version of itself;
// Just compare a small, blurred version of the input frame to all the KFs,
// choose the closest match, and then estimate a camera rotation by direct image
// minimisation.

#ifndef __RELOCALISER_H
#define __RELOCALISER_H
#define EIGEN_DONT_ALIGN_STATICALLY True

#include "ATANCamera.h"
#include "SmallBlurryImage.h"
#include "Map.h"
#include "RT.h"

class Relocaliser {
public:
  Relocaliser(Map &map, const ATANCamera &camera);
  bool AttemptRecovery(KeyFrame &k);
  mySE3 BestPose();
  
protected:
  void ScoreKFs(KeyFrame &kCurrentF);
  Map &mMap;
  ATANCamera mCamera;
  int mnBest;
  double mdBestScore;
  mySE2 mse2_eigen;
  mySE3 mse3Best;

};
#endif









