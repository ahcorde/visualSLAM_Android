// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited
#ifndef __TRACKERDATA_H
#define __TRACKERDATA_H
#define EIGEN_DONT_ALIGN_STATICALLY True

#include "PatchFinder.h"
#include "ATANCamera.h"

// This class contains all the intermediate results associated with
// a map-point that the tracker keeps up-to-date. TrackerData
// basically handles all the tracker's point-projection jobs,
// and also contains the PatchFinder which does the image search.
// It's very code-heavy for an h-file (it's a bunch of methods really)
// but it's only included from Tracker.cc!

Eigen::Vector2d myProject_TrackerData(Eigen::Vector3d& v) {
  Eigen::Vector2d result;
  result(0) = v(0)/v(2);
  result(1) = v(1)/v(2);

  return result;
}

Eigen::Vector4d myUnproject_TrackerData(Eigen::Vector3d& v) {
  Eigen::Vector4d result;
  result(0) = v(0);
  result(1) = v(1);
  result(2) = v(2);
  result(3) = 1.0;

  return result;
}

struct TrackerData
{
TrackerData(MapPoint *pMapPoint) 
: Point(*pMapPoint)
  {
    m26Jacobian.resize(2, 6);
  }
  
  MapPoint &Point;
  PatchFinder Finder;
  
  // Projection itermediates:
  Eigen::Vector3d v3Cam;        // Coords in current cam frame
  Eigen::Vector2d v2ImPlane;    // Coords in current cam z=1 plane
  Eigen::Vector2d v2Image;      // Pixel coords in LEVEL0
  Eigen::Matrix2d m2CamDerivs;  // Camera projection derivs
  bool bInImage;        
  bool bPotentiallyVisible;
  
  int nSearchLevel;
  bool bSearched;
  bool bFound;
  bool bDidSubPix;
  Eigen::Vector2d v2Found;      // Pixel coords of found patch (L0)
  double dSqrtInvNoise;   // Only depends on search level..
  
  
  // Stuff for pose update:
  Eigen::Vector2d v2Error_CovScaled;
  Eigen::MatrixXd m26Jacobian;   // Jacobian wrt camera position

  // Project point into image given certain pose and camera.
  // This can bail out at several stages if the point
  // will not be properly in the image.
  inline void Project(const mySE3& se3CFromW,  ATANCamera &Cam) {
    bInImage = bPotentiallyVisible = false;
    v3Cam = se3CFromW * Point.v3WorldPos;

    if(v3Cam[2] < 0.001)
      return;
    v2ImPlane = myProject_TrackerData(v3Cam);

    if(v2ImPlane.dot(v2ImPlane) > Cam.LargestRadiusInImage() * Cam.LargestRadiusInImage())
      return;
    v2Image = Cam.Project(v2ImPlane);
    if(Cam.Invalid())
      return;
    
    if(v2Image(0) < 0 || v2Image(1) < 0 || v2Image(0) > irImageSize(0) || v2Image(1) > irImageSize(1))
      return;
    bInImage = true;
  }
  
  // Get the projection derivatives (depend only on the camera.)
  // This is called Unsafe because it depends on the camera caching 
  // results from the previous projection:
  // Only do this right after the same point has been projected!
  inline void GetDerivsUnsafe(ATANCamera &Cam) 
  {
    m2CamDerivs = Cam.GetProjectionDerivs_Eigen();
  }
  
  // Does projection and gets camera derivs all in one.
  inline void ProjectAndDerivs(mySE3 &se3, ATANCamera &Cam) {
    Project(se3, Cam);
    if(bFound)
      GetDerivsUnsafe(Cam);
  }
  
  // Jacobian of projection W.R.T. the camera position
  // I.e. if  p_cam = SE3Old * p_world, 
  //         SE3New = SE3Motion * SE3Old
  inline void CalcJacobian()
  {
    double dOneOverCameraZ = 1.0 / v3Cam(2);

    for(int m=0; m<6; m++){
        Eigen::Vector4d v4Motion = mySE3::generator_field(m, myUnproject_TrackerData(v3Cam) );

        Eigen::Vector2d v2CamFrameMotion;
        v2CamFrameMotion[0] = (v4Motion[0] - v3Cam(0) * v4Motion[2] * dOneOverCameraZ) * dOneOverCameraZ;
        v2CamFrameMotion[1] = (v4Motion[1] - v3Cam(1) * v4Motion[2] * dOneOverCameraZ) * dOneOverCameraZ;

        Eigen::Vector2d aux = m2CamDerivs*v2CamFrameMotion;

        m26Jacobian(0, m) = aux(0);
        m26Jacobian(1, m) = aux(1);
    }
  }
  
  // Sometimes in tracker instead of reprojecting, just update the error linearly!
  inline void LinearUpdate(const Eigen::VectorXd &v6) {
    assert(v6.rows() == 6);

    Eigen::Vector2d v = m26Jacobian * v6;

    v2Image += v;
  }
  
  // This static member is filled in by the tracker and allows in-image checks in this class above.
  static Eigen::Vector2d irImageSize;
};






#endif




