#include "ATANCamera.h"
#include <iostream>

using namespace std;

ATANCamera::ATANCamera(string sName)
{
  // The camera name is used to find the camera's parameters in a GVar.
  msName = sName;


  mvDefaultParams.resize(NUMTRACKERCAMPARAMETERS);
  mvDefaultParams(0) = 0.5;
  mvDefaultParams(1) = 0.75;
  mvDefaultParams(2) = 0.5;
  mvDefaultParams(3) = 0.5;
  mvDefaultParams(4) = 0.1;

  mgvvCameraParams.resize(NUMTRACKERCAMPARAMETERS);
  mgvvCameraParams(0) = 0.841906;
  mgvvCameraParams(1) = 1.10893;
  mgvvCameraParams(2) = 0.505171;
  mgvvCameraParams(3) = 0.470265;
  mgvvCameraParams(4) = -0.0133843;

  mvImageSize(0) = 640.0;
  mvImageSize(1) = 480.0;
  RefreshParams();
}

void ATANCamera::SetImageSize(Eigen::Vector2d vImageSize)
{
  mvImageSize = vImageSize;
  RefreshParams();
}

void ATANCamera::RefreshParams() 
{
  // This updates internal member variables according to the current camera parameters,
  // and the currently selected target image size.
  //
  
  // First: Focal length and image center in pixel coordinates
  mvFocal(0) = mvImageSize(0)* (mgvvCameraParams)[0];
  mvFocal(1) = mvImageSize(1) * (mgvvCameraParams)[1];
  mvCenter(0) = mvImageSize(0) * (mgvvCameraParams)[2] - 0.5;
  mvCenter(1) = mvImageSize(1) * (mgvvCameraParams)[3] - 0.5;
  
  // One over focal length
  mvInvFocal(0) = 1.0 / mvFocal(0);
  mvInvFocal(1) = 1.0 / mvFocal(1);

  // Some radial distortion parameters..
  mdW =  (mgvvCameraParams)[4];
  if(mdW != 0.0){
      md2Tan = 2.0 * tan(mdW / 2.0);
      mdOneOver2Tan = 1.0 / md2Tan;
      mdWinv = 1.0 / mdW;
      mdDistortionEnabled = 1.0;
  }else{
      mdWinv = 0.0;
      md2Tan = 0.0;
      mdDistortionEnabled = 0.0;
  }
  
  // work out biggest radius in image
  Eigen::Vector2d v2;


  int m1 = (mgvvCameraParams)[2];
  int m2  = 1.0 - (mgvvCameraParams)[2];

  v2(0)= std::max(m1, m2) / (mgvvCameraParams)[0];

  m1 = (mgvvCameraParams)[3];
  m2  = 1.0 - (mgvvCameraParams)[3];

  v2(1)= std::max(m1, m2) / (mgvvCameraParams)[1];
  mdLargestRadius = invrtrans(sqrt(v2.dot(v2)));
  
  // At what stage does the model become invalid?
  mdMaxR = 1.5 * mdLargestRadius; // (pretty arbitrary)

  // work out world radius of one pixel
  // (This only really makes sense for square-ish pixels)
  {
    Eigen::Vector2d v2Center = UnProject(mvImageSize / 2);
    Eigen::Vector2d v2RootTwoAway = UnProject(mvImageSize / 2 + Eigen::Vector2d(1,1));
    Eigen::Vector2d v2Diff = v2Center - v2RootTwoAway;
    mdOnePixelDist = sqrt(v2Diff.dot(v2Diff)) / sqrt(2.0);
  }
  
  // Work out the linear projection values for the UFB
  {
    // First: Find out how big the linear bounding rectangle must be
    vector<Eigen::Vector2d > vv2Verts;
    vv2Verts.push_back(UnProject(Eigen::Vector2d( -0.5, -0.5)));
    vv2Verts.push_back(UnProject(Eigen::Vector2d( mvImageSize(0)-0.5, -0.5)));
    vv2Verts.push_back(UnProject(Eigen::Vector2d( mvImageSize(0)-0.5, mvImageSize(1)-0.5)));
    vv2Verts.push_back(UnProject(Eigen::Vector2d( -0.5, mvImageSize(1)-0.5)));
    Eigen::Vector2d v2Min = vv2Verts[0];
    Eigen::Vector2d v2Max = vv2Verts[0];
    for(int i=0; i<4; i++){
      if(vv2Verts[i](0) < v2Min(0))
          v2Min(0) = vv2Verts[i](0);

      if(vv2Verts[i](1) < v2Min(1))
        v2Min(1) = vv2Verts[i](1);

      if(vv2Verts[i](1) > v2Max(1))
          v2Max(1) = vv2Verts[i](1);


      if(vv2Verts[i](0) > v2Max(0))
          v2Max(0) = vv2Verts[i](0);
    }
    mvImplaneTL = v2Min;
    mvImplaneBR = v2Max;
    
    // Store projection parameters to fill this bounding box
    Eigen::Vector2d v2Range = v2Max - v2Min;
    mvUFBLinearInvFocal = v2Range;
    mvUFBLinearFocal(0) = 1.0 / mvUFBLinearInvFocal(0);
    mvUFBLinearFocal(1) = 1.0 / mvUFBLinearInvFocal(1);
    mvUFBLinearCenter(0) = -1.0 * v2Min(0) * mvUFBLinearFocal(0);
    mvUFBLinearCenter(1) = -1.0 * v2Min(1) * mvUFBLinearFocal(1);
  }
  
}

//// Project from the camera z=1 plane to image pixels,
//// while storing intermediate calculation results in member variables
Eigen::Vector2d ATANCamera::Project(const Eigen::Vector2d& vCam) {
  mvLastCam = vCam;
  mdLastR = sqrt(vCam.squaredNorm());
  mbInvalid = (mdLastR > mdMaxR);
  mdLastFactor = rtrans_factor(mdLastR);
  mdLastDistR = mdLastFactor * mdLastR;
  mvLastDistCam =  mvLastCam*mdLastFactor;

  mvLastIm(0) = mvCenter(0) + mvFocal(0) * mvLastDistCam(0);
  mvLastIm(1) = mvCenter(1) + mvFocal(1) * mvLastDistCam(1);

  return mvLastIm;
}

// Un-project from image pixel coords to the camera z=1 plane
// while storing intermediate calculation results in member variables
Eigen::Vector2d ATANCamera::UnProject(const Eigen::Vector2d& v2Im)
{
    mvLastIm = v2Im;
    mvLastDistCam(0) = (mvLastIm(0) - mvCenter(0)) * mvInvFocal(0);
    mvLastDistCam(1) = (mvLastIm(1) - mvCenter(1)) * mvInvFocal(1);
    mdLastDistR = sqrt(mvLastDistCam.squaredNorm());
    mdLastR = invrtrans(mdLastDistR);
    double dFactor;
    if(mdLastDistR > 0.01)
    dFactor =  mdLastR / mdLastDistR;
    else
    dFactor = 1.0;
    mdLastFactor = 1.0 / dFactor;
    mvLastCam =  mvLastDistCam * dFactor;
    return mvLastCam;
}

// Utility function for easy drawing with OpenGL
// C.f. comment in top of ATANCamera.h
Eigen::Matrix4d ATANCamera::MakeUFBLinearFrustumMatrix(double near, double far)
{
    Eigen::Matrix4d m4 = Eigen::Matrix4d::Zero();


  double left = mvImplaneTL(0) * near;
  double right = mvImplaneBR(0) * near;
  double top = mvImplaneTL(1) * near;
  double bottom = mvImplaneBR(1) * near;
  
  // The openGhelL frustum manpage is A PACK OF LIES!!
  // Two of the elements are NOT what the manpage says they should be.
  // Anyway, below code makes a frustum projection matrix
  // Which projects a RHS-coord frame with +z in front of the camera
  // Which is what I usually want, instead of glFrustum's LHS, -z idea.
  m4(0, 0) = (2 * near) / (right - left);
  m4(1, 1) = (2 * near) / (top - bottom);
  
  m4(0, 2) = (right + left) / (left - right);
  m4(1, 2) = (top + bottom) / (bottom - top);
  m4(2, 2) = (far + near) / (far - near);
  m4(3, 2) = 1;
  
  m4(2, 3) = 2*near*far / (near - far);

  return m4;
}



Eigen::Matrix2d ATANCamera::GetProjectionDerivs_Eigen()
{
  // get the derivative of image frame wrt camera z=1 frame at the last computed projection
  // in the form (d im1/d cam1, d im1/d cam2)
  //             (d im2/d cam1, d im2/d cam2)

  double dFracBydx;
  double dFracBydy;

  double &k = md2Tan;
  double x = mvLastCam(0);
  double y = mvLastCam(1);
  double r = mdLastR * mdDistortionEnabled;

  if(r < 0.01){
      dFracBydx = 0.0;
      dFracBydy = 0.0;
  }else{
      dFracBydx =
    mdWinv * (k * x) / (r*r*(1 + k*k*r*r)) - x * mdLastFactor / (r*r);
      dFracBydy =
    mdWinv * (k * y) / (r*r*(1 + k*k*r*r)) - y * mdLastFactor / (r*r);
  }

  Eigen::Matrix2d m2Derivs_eigen;


  m2Derivs_eigen(0, 0) = mvFocal(0) * (dFracBydx * x + mdLastFactor);
  m2Derivs_eigen(1, 0) = mvFocal(1) * (dFracBydx * y);
  m2Derivs_eigen(0, 1) = mvFocal(0) * (dFracBydy * x);
  m2Derivs_eigen(1, 1) = mvFocal(1) * (dFracBydy * y + mdLastFactor);

  return m2Derivs_eigen;
}

Eigen::Vector2d ATANCamera::UFBProject(const Eigen::Vector2d& vCam)
{
  // Project from camera z=1 plane to UFB, storing intermediate calc results.
  mvLastCam = vCam;
  mdLastR = sqrt(vCam.squaredNorm());
  mbInvalid = (mdLastR > mdMaxR);
  mdLastFactor = rtrans_factor(mdLastR);
  mdLastDistR = mdLastFactor * mdLastR;
  mvLastDistCam = mvLastCam*mdLastFactor;

  mvLastIm(0) = (mgvvCameraParams)[2]  + (mgvvCameraParams)[0] * mvLastDistCam(0);
  mvLastIm(1) = (mgvvCameraParams)[3]  + (mgvvCameraParams)[1] * mvLastDistCam(1);

  return mvLastIm;
}

Eigen::Vector2d ATANCamera::UFBUnProject(const Eigen::Vector2d& v2Im)
{
  mvLastIm = v2Im;
  mvLastDistCam(0) = (mvLastIm(0) - (mgvvCameraParams)[2]) / (mgvvCameraParams)[0];
  mvLastDistCam(1) = (mvLastIm(1) - (mgvvCameraParams)[3]) / (mgvvCameraParams)[1];
  mdLastDistR = sqrt(mvLastDistCam.squaredNorm());
  mdLastR = invrtrans(mdLastDistR);
  double dFactor;
  if(mdLastDistR > 0.01)
    dFactor =  mdLastR / mdLastDistR;
  else
    dFactor = 1.0;
  mdLastFactor = 1.0 / dFactor;
  mvLastCam = mvLastDistCam*dFactor;
  return mvLastCam;
}
