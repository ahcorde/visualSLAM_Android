// N-th implementation of a camera model
// GK 2007
// 
// This one uses the ``FOV'' distortion model of
// Deverneay and Faugeras, Straight lines have to be straight, 2001
//
// BEWARE: This camera model caches intermediate results in member variables
// Some functions therefore depend on being called in order: i.e.
// GetProjectionDerivs() uses data stored from the last Project() or UnProject()
// THIS MEANS YOU MUST BE CAREFUL WITH MULTIPLE THREADS
// Best bet is to give each thread its own version of the camera!
//
// Camera parameters are stored in a GVar, but changing the gvar has no effect
// until the next call to RefreshParams() or SetImageSize().
//
// Pixel conventions are as follows:
// For Project() and Unproject(),
// round pixel values - i.e. (0.0, 0.0) - refer to pixel centers
// I.e. the top left pixel in the image covers is centered on (0,0)
// and covers the area (-.5, -.5) to (.5, .5)
//
// Be aware that this is not the same as what opengl uses but makes sense
// for acessing pixels using ImageRef, especially ir_rounded.
//
// What is the UFB?
// This is for projecting the visible image area
// to a unit square coordinate system, with the top-left at 0,0,
// and the bottom-right at 1,1
// This is useful for rendering into textures! The top-left pixel is NOT
// centered at 0,0, rather the top-left corner of the top-left pixel is at 
// 0,0!!! This is the way OpenGL thinks of pixel coords.
// There's the Linear and the Distorting version - 
// For the linear version, can use 
// glMatrixMode(GL_PROJECTION); glLoadIdentity();
// glMultMatrix(Camera.MakeUFBLinearFrustumMatrix(near,far));
// To render un-distorted geometry with full frame coverage.
//

#ifndef __ATAN_CAMERA_H
#define __ATAN_CAMERA_H
#define EIGEN_DONT_ALIGN_STATICALLY True

#include <cmath>
#include "eigen3/Eigen/Dense"


#include <vector>

#define NUMTRACKERCAMPARAMETERS 5

class CameraCalibrator;
class CalibImage;

// The parameters are:
// 0 - normalized x focal length
// 1 - normalized y focal length
// 2 - normalized x offset
// 3 - normalized y offset
// 4 - w (distortion parameter)

class ATANCamera {
 public:
  ATANCamera(std::string sName);

  // Image size get/set: updates the internal projection params to that target image size.
  void SetImageSize(Eigen::Vector2d v2ImageSize);

  inline Eigen::Vector2d GetImageSize()
  {
      return mvImageSize;
  }
  void RefreshParams();
  
  // Various projection functions

  Eigen::Vector2d Project(const Eigen::Vector2d& vCam);
  Eigen::Vector2d UnProject(const Eigen::Vector2d& imframe); // Inverse operation

  Eigen::Vector2d UFBProject(const Eigen::Vector2d& vCam);
  Eigen::Vector2d UFBUnProject(const Eigen::Vector2d& v2Im);
  inline Eigen::Vector2d  UFBLinearUnProject(const Eigen::Vector2d & fbframe);
  inline Eigen::Vector2d  UFBLinearProject(const Eigen::Vector2d & camframe);

  
  Eigen::Matrix2d GetProjectionDerivs_Eigen(); // Projection jacobian

  inline bool Invalid() {  return mbInvalid;}
  inline double LargestRadiusInImage() {  return mdLargestRadius; }
  inline double OnePixelDist() { return mdOnePixelDist; }
  
  // The z=1 plane bounding box of what the camera can see
  inline Eigen::Vector2d ImplaneTL();
  inline Eigen::Vector2d ImplaneBR();

  // OpenGL helper function
  Eigen::Matrix4d MakeUFBLinearFrustumMatrix(double near, double far);

  // Feedback for Camera Calibrator
  double PixelAspectRatio() { return mvFocal(1) / mvFocal(0);}
  
  
  // Useful for gvar-related reasons (in case some external func tries to read the camera params gvar, and needs some defaults.)
  Eigen::VectorXd mvDefaultParams;
  
 protected:
  Eigen::VectorXd mgvvCameraParams; // The actual camera parameters
  

  // Cached from the last project/unproject:
  Eigen::Vector2d mvLastCam;      // Last z=1 coord
  Eigen::Vector2d mvLastIm;       // Last image/UFB coord
  Eigen::Vector2d mvLastDistCam;  // Last distorted z=1 coord
  double mdLastR;           // Last z=1 radius
  double mdLastDistR;       // Last z=1 distorted radius
  double mdLastFactor;      // Last ratio of z=1 radii
  bool mbInvalid;           // Was the last projection invalid?
  
  // Cached from last RefreshParams:
  double mdLargestRadius; // Largest R in the image
  double mdMaxR;          // Largest R for which we consider projection valid
  double mdOnePixelDist;  // z=1 distance covered by a single pixel offset (a rough estimate!)
  double md2Tan;          // distortion model coeff
  double mdOneOver2Tan;   // distortion model coeff
  double mdW;             // distortion model coeff
  double mdWinv;          // distortion model coeff
  double mdDistortionEnabled; // One or zero depending on if distortion is on or off.
  Eigen::Vector2d mvCenter;     // Pixel projection center
  Eigen::Vector2d mvFocal;      // Pixel focal length
  Eigen::Vector2d mvInvFocal;   // Inverse pixel focal length
  Eigen::Vector2d mvImageSize;
  Eigen::Vector2d mvUFBLinearFocal;
  Eigen::Vector2d mvUFBLinearInvFocal;
  Eigen::Vector2d mvUFBLinearCenter;
  Eigen::Vector2d mvImplaneTL;
  Eigen::Vector2d mvImplaneBR;
  
  // Radial distortion transformation factor: returns ration of distorted / undistorted radius.
  inline double rtrans_factor(double r)
  {
    if(r < 0.001 || mdW == 0.0)
      return 1.0;
    else 
      return (mdWinv* atan(r * md2Tan) / r);
  }

  // Inverse radial distortion: returns un-distorted radius from distorted.
  inline double invrtrans(double r)
  {
    if(mdW == 0.0)
      return r;
    return(tan(r * mdW) * mdOneOver2Tan);
  }
  
  std::string msName;

  friend class CameraCalibrator;   // friend declarations allow access to calibration jacobian and camera update function.
  friend class CalibImage;
};

// Some inline projection functions:
inline Eigen::Vector2d ATANCamera::UFBLinearProject(const Eigen::Vector2d & camframe)
{
  Eigen::Vector2d v2Res;
  v2Res(0) = camframe(0) * mvUFBLinearFocal(0) + mvUFBLinearCenter(0);
  v2Res(1) = camframe(1) * mvUFBLinearFocal(1) + mvUFBLinearCenter(1);
  return v2Res;
}

inline Eigen::Vector2d  ATANCamera::UFBLinearUnProject(const Eigen::Vector2d & fbframe)
{
  Eigen::Vector2d v2Res;
  v2Res(0) = (fbframe(0) - mvUFBLinearCenter(0)) * mvUFBLinearInvFocal(0);
  v2Res(1) = (fbframe(1) - mvUFBLinearCenter(1)) * mvUFBLinearInvFocal(1);
  return v2Res;
}


#endif

