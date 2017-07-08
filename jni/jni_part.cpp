#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>

#include "vision/cvfast.h"
#include "KeyFrame.h"
#include "ATANCamera.h"
#include "Map.h"
#include "MapMaker.h"
#include "Tracker.h"

using namespace std;
using namespace cv;

class SystemPTAM {

public:
	  Map *mpMap;
	  MapMaker *mpMapMaker;
	  Tracker *mpTracker;
	  ATANCamera *mpCamera;

	  bool mbDone;


	public: SystemPTAM()
	{
	  Eigen::VectorXd vTest(NUMTRACKERCAMPARAMETERS);

	  vTest(0) = 0.841906 ;
	  vTest(1) = 1.10893 ;
	  vTest(2) = 0.505171 ;
	  vTest(3) = 0.470265 ;
	  vTest(4) = -0.0133843;

	  mpCamera = new ATANCamera("Camera");

	  mpMap = new Map;
	  mpMapMaker = new MapMaker(*mpMap, *mpCamera);
	  mpTracker = new Tracker(800, 480, *mpCamera, *mpMap, *mpMapMaker);
//	  mpTracker = new Tracker(1024, 552, *mpCamera, *mpMap, *mpMapMaker);

	  mbDone = false;

	};


	public: void onTouchScreen(){
        mpTracker->mbUserPressedSpacebar = true;
	}

	public: ~SystemPTAM()
	{

	};


	public: void update(cv::Mat& mimFrameBWCV, cv::Mat& mimFrameRGBCV)
	{
	    static bool bFirstFrame = true;
	    if(bFirstFrame){
	        bFirstFrame = false;
	    }

	    bool bDrawMap = false;
	    bool bDrawAR = false;
	    mpTracker->TrackFrame(mimFrameBWCV, mimFrameRGBCV, !bDrawAR && !bDrawMap);


	};

//	public: void update(long matAddrGr, long matAddrRgba);

};

#include <jni.h>
#ifndef _Included_Test
#define _Included_Test
#ifdef __cplusplus
extern "C" {
#endif

	JNIEXPORT void JNICALL Java_vision_ar_monoslam_MainActivity_FindFeatures(JNIEnv*, jobject, jlong addrGray, jlong addrRgba);

	JNIEXPORT void JNICALL Java_vision_ar_monoslam_MainActivity_FindFeatures(JNIEnv*, jobject, jlong addrGray, jlong addrRgba)
	{
//		Mat& mGr  = *(Mat*)addrGray;
//		Mat& mRgb = *(Mat*)addrRgba;
//		std::vector<Eigen::Vector2d> vCorners;
//        cvCornerFast_10(mGr, vCorners, 10);
//		for( unsigned int i = 0; i < vCorners.size(); i++ ){
//			const Eigen::Vector2d& kp = vCorners[i];
//			circle(mRgb, Point(kp(0), kp(1)), 10, Scalar(255,0,0));
//
//		}
//
//		KeyFrame mCurrentKF;
//	    mCurrentKF.mMeasurements.clear();
//	    mCurrentKF.MakeKeyFrame_Lite(mGr, mRgb);
//
//		return vCorners.size();
//		FastFeatureDetector detector(50);
//		detector.detect(mGr, v);
//		for( unsigned int i = 0; i < v.size(); i++ )
//		{
//			const KeyPoint& kp = v[i];
//			circle(mRgb, Point(kp.pt.x, kp.pt.y), 10, Scalar(255,0,0,255));
//		}
	}

	//----------------------------------//

	JNIEXPORT jlong JNICALL Java_vision_ar_monoslam_SystemPTAM_native_1createTest(JNIEnv *, jobject)
	{
		SystemPTAM *test=new SystemPTAM();
		return reinterpret_cast<jlong>(test);
	}
	JNIEXPORT void JNICALL Java_vision_ar_monoslam_SystemPTAM_native_1disposeTest(JNIEnv *, jobject, jlong cptr)
	{
		SystemPTAM *test=reinterpret_cast<SystemPTAM*>(cptr);
		delete test;
	}

	JNIEXPORT void JNICALL Java_vision_ar_monoslam_SystemPTAM_native_1touchScreen(JNIEnv *, jobject, jlong cptr)
	{
		SystemPTAM *test=reinterpret_cast<SystemPTAM*>(cptr);
		test->onTouchScreen();
	}


	JNIEXPORT int JNICALL Java_vision_ar_monoslam_SystemPTAM_native_1update(JNIEnv *, jobject, jlong cptr, jlong addrGray, jlong addrRgba)
	{
		Mat& mGr  = *(Mat*)addrGray;
		Mat& mRgb = *(Mat*)addrRgba;

		SystemPTAM *test=reinterpret_cast<SystemPTAM*>(cptr);

//		test->KF.mMeasurements.clear();
//		test->KF.MakeKeyFrame_Lite(mGr, mRgb);

		test->update(mGr, mRgb);

		return 0;
	}

#ifdef __cplusplus
}
#endif
#endif
