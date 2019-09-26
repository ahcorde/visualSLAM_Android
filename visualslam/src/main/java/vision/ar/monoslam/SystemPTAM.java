package vision.ar.monoslam;

import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;

public class SystemPTAM {

	private native long native_createTest();

	private native void native_disposeTest(long cptr);

	//    private native void native_setA(long cptr, int value);
//    private native int native_getA(long cptr);
	private native void native_touchScreen(long cptr);

	private native int native_update(long cptr, long addrGray, long addrRgba);

	static {
		System.loadLibrary("mixed_sample");
	}

	private long _cptr;

	public SystemPTAM() {
		_cptr = native_createTest();
	}

	public void update(CvCameraViewFrame inputFrame) {
		System.out.println("nChannels java: " + inputFrame.rgba().channels());
		System.out.println("nChannels: " + native_update(_cptr,
				inputFrame.gray().getNativeObjAddr(),
				inputFrame.rgba().getNativeObjAddr()));
	}

	public void onTouchScreen() {
		native_touchScreen(_cptr);
	}

	//    public void setA(int valor) {
//    	native_setA(_cptr,valor);
//    }
//    public int getA() {
//    	return native_getA(_cptr);
//    }
	protected void finalize() throws Throwable {
		native_disposeTest(_cptr);
		super.finalize();
	}
}
