LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
OPENCV_MK_PATH := C:\Users\Pascal\AndroidstudioProjects\OpenCV-3.4.1-android-sdk\sdk\native\jni\OpenCV.mk
include ${OPENCV_MK_PATH}

LOCAL_MODULE    := mixed_sample
LOCAL_SRC_FILES := jni_part.cpp Tracker.cc MiniPatch.cc Relocaliser.cc PatchFinder.cc MapPoint.cc HomographyInit.cc Bundle.cc MapMaker.cc Map.cc vision/cvfast.cpp vision/ImageHandler.cpp KeyFrame.cc SmallBlurryImage.cc ATANCamera.cc
LOCAL_LDLIBS +=  -llog -ldl

include $(BUILD_SHARED_LIBRARY)
#LOCAL_C_INCLUDES += /opt/local/include/eigen3/