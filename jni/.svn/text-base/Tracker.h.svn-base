//-*- C++ -*-
// Copyright 2008 Isis Innovation Limited
// 
// This header declares the Tracker class.
// The Tracker is one of main components of the system,
// and is responsible for determining the pose of a camera
// from a video feed. It uses the Map to track, and communicates 
// with the MapMaker (which runs in a different thread)
// to help construct this map.
//
// Initially there is no map, so the Tracker also has a mode to 
// do simple patch tracking across a stereo pair. This is handled 
// by the TrackForInitialMap() method and associated sub-methods. 
// Once there is a map, TrackMap() is used.
//
// Externally, the tracker should be used by calling TrackFrame()
// with every new input video frame. This then calls either 
// TrackForInitialMap() or TrackMap() as appropriate.
//

#ifndef __TRACKER_H
#define __TRACKER_H
#define EIGEN_DONT_ALIGN_STATICALLY True

#include <sstream>
#include <vector>
#include <list>
#include <opencv2/core/core.hpp>
#include "MapMaker.h"
#include "ATANCamera.h"
#include "MiniPatch.h"
#include "Relocaliser.h"
#include "myWLS.h"
#include "RT.h"

class TrackerData;
struct Trail {   // This struct is used for initial correspondences of the first stereo pair.
  MiniPatch mPatch;
  Eigen::Vector2d irCurrentPos;
  Eigen::Vector2d irInitialPos;
};

class Tracker {
public:
  Tracker(int width, int heigth, const ATANCamera &c, Map &m, MapMaker &mm);

  struct compareParticulaFunctor : public std::binary_function < pair<double,Eigen::Vector2d>, pair<double,Eigen::Vector2d>, bool>
  {
      bool operator()( pair<double,Eigen::Vector2d> lhs, pair<double,Eigen::Vector2d> rhs) {
          return (lhs.first > rhs.first);
      }
  };

  // TrackFrame is the main working part of the tracker: call this every frame.
  void TrackFrame(cv::Mat &imFrame, cv::Mat &imageColor, bool bDraw);
  void drawFast(cv::Mat &image);

  inline mySE3 GetCurrentPose() { return mse3CamFromWorld;}
  
  // Gets messages to be printed on-screen for the user.
  std::string GetMessageForUser();
  
  void Reset();
  bool mbDraw;                    // Should the tracker draw anything to OpenGL?
  void RenderGrid(cv::Mat &imageColor);              // Draws the reference grid

  inline int get_mnInitialStage()
  {
      return mnInitialStage;
  }

  public:  Relocaliser* mRelocaliser;       // Relocalisation module


protected:
  KeyFrame mCurrentKF;            // The current working frame as a keyframe struct
  
  // The major components to which the tracker needs access:
  Map &mMap;                      // The map, consisting of points and keyframes
  MapMaker &mMapMaker;            // The class which maintains the map
  ATANCamera mCamera;             // Projection model

  Eigen::Vector2d mirSize;          // Image size of whole image
  
                 // Restart from scratch. Also tells the mapmaker to reset itself.

  // The following members are used for initial map tracking (to get the first stereo pair and correspondences):
  void TrackForInitialMap(cv::Mat &imageColor);      // This is called by TrackFrame if there is not a map yet.
  enum {TRAIL_TRACKING_NOT_STARTED, 
	TRAIL_TRACKING_STARTED, 
	TRAIL_TRACKING_COMPLETE} mnInitialStage;  // How far are we towards making the initial map?

  void TrailTracking_Start();     // First frame of initial trail tracking. Called by TrackForInitialMap.
  int  TrailTracking_Advance(cv::Mat &imageColor);   // Steady-state of initial trail tracking. Called by TrackForInitialMap.
  std::list<Trail> mlTrails;      // Used by trail tracking
  KeyFrame mFirstKF;              // First of the stereo pair
  KeyFrame mPreviousFrameKF;      // Used by trail tracking to check married matches
  
  // Methods for tracking the map once it has been made:
  void TrackMap(cv::Mat &imageColor);                // Called by TrackFrame if there is a map.
  void AssessTrackingQuality();   // Heuristics to choose between good, poor, bad.
  void ApplyMotionModel();        // Decaying velocity motion model applied prior to TrackMap
  void UpdateMotionModel();       // Motion model is updated after TrackMap
  int SearchForPoints(std::vector<TrackerData*> &vTD, int nRange, int nFineIts);  // Finds points in the image
  Eigen::VectorXd CalcPoseUpdate(std::vector<TrackerData*> vTD, double dOverrideSigma = 0.0, bool bMarkOutliers = false); // Updates pose from found points.
  mySE3 mse3StartPos;               // What the camera pose was at the start of the frame.
  mySE3 mse3CamFromWorld;           // Camera pose: this is what the tracker updates every frame.

  Eigen::VectorXd mv6CameraVelocity_eigen;    // Motion model
  double mdVelocityMagnitude;     // Used to decide on coarse tracking
  double mdMSDScaledVelocityMagnitude; // Velocity magnitude scaled by relative scene depth.
  bool mbDidCoarse;               // Did tracking use the coarse tracking stage?
    
  // Interface with map maker:
  int mnFrame;                    // Frames processed since last reset
  int mnLastKeyFrameDropped;      // Counter of last keyframe inserted.
  void AddNewKeyFrame();          // Gives the current frame to the mapmaker to use as a keyframe
  
  // Tracking quality control:
  int manMeasAttempted[LEVELS];
  int manMeasFound[LEVELS];
  enum {BAD, DODGY, GOOD} mTrackingQuality;
  int mnLostFrames;
  
  // Relocalisation functions:
  bool AttemptRecovery();         // Called by TrackFrame if tracking is lost.
  bool mbJustRecoveredSoUseCoarse;// Always use coarse tracking after recovery!

  // Frame-to-frame motion init:
  SmallBlurryImage *mpSBILastFrame;
  SmallBlurryImage *mpSBIThisFrame;
  void CalcSBIRotation();
  Eigen::VectorXd mv6SBIRot_eigen;
  bool mbUseSBIInit;
  
  // User interaction for initial tracking:
public:
  bool mbUserPressedSpacebar;
  bool mbUserPressedReset;

protected:
  std::ostringstream mMessageForUser;
};

#endif






