#include "MapPoint.h"
#include "KeyFrame.h"

void MapPoint::RefreshPixelVectors() {
  KeyFrame &k = *pPatchSourceKF;
  
  // Find patch pos in KF camera coords
  // Actually this might not exactly correspond to the patch pos!
  // Treat it as a general point on the plane.
  Eigen::Vector3d v3PlanePoint_C = k.se3CfromW * v3WorldPos;

  // Find the height of this above the plane.
  // Assumes the normal is  pointing toward the camera.
  double dCamHeight = fabs(v3PlanePoint_C.dot(v3Normal_NC));

  double dPixelRate = fabs(v3Center_NC.dot(v3Normal_NC));

  double dOneRightRate = fabs(v3OneRightFromCenter_NC.dot( v3Normal_NC));
  double dOneDownRate = fabs(v3OneDownFromCenter_NC.dot( v3Normal_NC));

  // Find projections onto plane
  Eigen::Vector3d v3CenterOnPlane_C = v3Center_NC * dCamHeight / dPixelRate;
  Eigen::Vector3d v3OneRightOnPlane_C = v3OneRightFromCenter_NC * dCamHeight / dOneRightRate;
  Eigen::Vector3d v3OneDownOnPlane_C = v3OneDownFromCenter_NC * dCamHeight / dOneDownRate;

  // Find differences of these projections in the world frame
  v3PixelRight_W = k.se3CfromW.get_rotation().inverse() * (v3OneRightOnPlane_C - v3CenterOnPlane_C);
  v3PixelDown_W = k.se3CfromW.get_rotation().inverse() * (v3OneDownOnPlane_C - v3CenterOnPlane_C);
}  
