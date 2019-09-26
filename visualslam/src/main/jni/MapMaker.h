// This header declares the MapMaker class
// MapMaker makes and maintains the Map struct
// Starting with stereo initialisation from a bunch of matches
// over keyframe insertion, continual bundle adjustment and 
// data-association refinement.
// MapMaker runs in its own thread, although some functions
// (notably stereo init) are called by the tracker and run in the 
// tracker's thread.

#ifndef __MAPMAKER_H
#define __MAPMAKER_H
#define EIGEN_DONT_ALIGN_STATICALLY True

#include "Map.h"
#include "KeyFrame.h"
#include "ATANCamera.h"
#include <queue>
#include "RT.h"
//#include <boost/thread.hpp>

// Each MapPoint has an associated MapMakerData class
// Where the mapmaker can store extra information

struct MapMakerData
{
  std::set<KeyFrame*> sMeasurementKFs;   // Which keyframes has this map point got measurements in?
  std::set<KeyFrame*> sNeverRetryKFs;    // Which keyframes have measurements failed enough so I should never retry?
  inline int GoodMeasCount()            
  {  return sMeasurementKFs.size(); }
};

// MapMaker dervives from QThread, so everything in void run() is its own thread.
class MapMaker  {
public:
  MapMaker(Map &m, const ATANCamera &cam);
  ~MapMaker();
  
  // Make a map from scratch. Called by the tracker.
  bool InitFromStereo(KeyFrame &kFirst, KeyFrame &kSecond, vector<pair<Eigen::Vector2d, Eigen::Vector2d> > &vMatches, mySE3 &se3TrackerPose);
  
  void AddKeyFrame(KeyFrame &k);   // Add a key-frame to the map. Called by the tracker.
  void RequestReset();   // Request that the we reset. Called by the tracker.
  bool ResetDone();      // Returns true if the has been done.
  int  QueueSize() { return mvpKeyFrameQueue.size() ;} // How many KFs in the queue waiting to be added?
  bool NeedNewKeyFrame(KeyFrame &kCurrent);            // Is it a good camera pose to add another KeyFrame?
  bool IsDistanceToNearestKeyFrameExcessive(KeyFrame &kCurrent);  // Is the camera far away from the nearest KeyFrame (i.e. maybe lost?)

  void run();               // The MapMaker thread code lives here

  
protected:
  
  Map &mMap;               // The map
  ATANCamera mCamera;      // Same as the tracker's camera: N.B. not a reference variable!

  // Functions for starting the map from scratch:
  mySE3 CalcPlaneAligner();
  void ApplyGlobalTransformationToMap(mySE3 se3NewFromOld);
  void ApplyGlobalScaleToMap(double dScale);
  
  // Map expansion functions:
  void AddKeyFrameFromTopOfQueue();  
  void ThinCandidates(KeyFrame &k, int nLevel);
  void AddSomeMapPoints(int nLevel);
  bool AddPointEpipolar(KeyFrame &kSrc, KeyFrame &kTarget, int nLevel, int nCandidate);
  // Returns point in ref frame B
  Eigen::Vector3d ReprojectPoint(mySE3 se3AfromB, const Eigen::Vector2d &v2A, const Eigen::Vector2d &v2B);
  
  // Bundle adjustment functions:
  void BundleAdjust(std::set<KeyFrame*>, std::set<KeyFrame*>, std::set<MapPoint*>, bool);
  void BundleAdjustAll();
  void BundleAdjustRecent();

  // Data association functions:
  int ReFindInSingleKeyFrame(KeyFrame &k);
  void ReFindFromFailureQueue();
  void ReFindNewlyMade();
  void ReFindAll();
  bool ReFind_Common(KeyFrame &k, MapPoint &p);
  void SubPixelRefineMatches(KeyFrame &k, int nLevel);
  
  // General Maintenance/Utility:
  void Reset();
  void HandleBadPoints();
  double DistToNearestKeyFrame(KeyFrame &kCurrent);
  double KeyFrameLinearDist(KeyFrame &k1, KeyFrame &k2);
  KeyFrame* ClosestKeyFrame(KeyFrame &k);
  std::vector<KeyFrame*> NClosestKeyFrames(KeyFrame &k, unsigned int N);
  void RefreshSceneDepth(KeyFrame *pKF);
  

  // GUI Interface:
  void GUICommandHandler(std::string sCommand, std::string sParams);
  struct Command {
      std::string sCommand;
      std::string sParams;
  };

  // Member variables:
  std::vector<KeyFrame*> mvpKeyFrameQueue;  // Queue of keyframes from the tracker waiting to be processed
  std::vector<std::pair<KeyFrame*, MapPoint*> > mvFailureQueue; // Queue of failed observations to re-find
  std::queue<MapPoint*> mqNewQueue;   // Queue of newly-made map points to re-find in other KeyFrames
  
  double mdWiggleScale;  // Metric distance between the first two KeyFrames (copied from GVar)
                         // This sets the scale of the map
  double mgvdWiggleScale;   // GVar for above
  double mdWiggleScaleDepthNormalized;  // The above normalized against scene depth, 
                                        // this controls keyframe separation
  
  bool mbBundleConverged_Full;    // Has global bundle adjustment converged?
  bool mbBundleConverged_Recent;  // Has local bundle adjustment converged?
  
  // Thread interaction signalling stuff
  bool mbResetRequested;   // A reset has been requested
  bool mbResetDone;        // The reset was done.
  bool mbBundleAbortRequested;      // We should stop bundle adjustment
  bool mbBundleRunning;             // Bundle adjustment is running
  bool mbBundleRunningIsRecent;     //    ... and it's a local bundle adjustment.

  
};

#endif


















