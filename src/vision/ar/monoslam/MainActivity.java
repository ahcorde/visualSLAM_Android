package vision.ar.monoslam;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.highgui.Highgui;

import android.os.Bundle;
import android.app.Activity;
import android.view.Menu;
import android.view.MotionEvent;
import android.view.View;
import android.view.View.OnTouchListener;
import android.view.WindowManager;

public class MainActivity extends Activity implements CvCameraViewListener2, OnTouchListener {

    private CameraBridgeViewBase mOpenCvCameraView;

    private Mat                  mRgba;
    private Mat                  mGray;

    private SystemPTAM prueba;
    
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
		
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        
        setContentView(R.layout.activity_main);
			
        mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.tutorial2_activity_surface_view);
        mOpenCvCameraView.setCvCameraViewListener(this);
        mOpenCvCameraView.SetCaptureFormat(Highgui.CV_CAP_ANDROID_COLOR_FRAME_RGBA);
        
        
    }
   
    private BaseLoaderCallback  mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS:
                {
                    prueba = new SystemPTAM();
                	               	
                    mOpenCvCameraView.enableView();
                    
                    mOpenCvCameraView.setOnTouchListener(MainActivity.this);

                    
                } break;
                default:
                {
                    super.onManagerConnected(status);
                } break;
            }
        }
    };
	
	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
//		getMenuInflater().inflate(R.menu.activity_main, menu);
		return true;
	}

	@Override
	public void onCameraViewStarted(int width, int height) {
        mRgba = new Mat(height, width, CvType.CV_8UC4);
        mGray = new Mat(height, width, CvType.CV_8UC1);
        
        System.out.println(height + " " + width);

	}

	@Override
	public void onCameraViewStopped() {
        mRgba.release();	
        mGray.release();
	}
	
	
    @Override
    public void onPause()
    {
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
        super.onPause();
    }
    
    @Override
    public void onDestroy() {
        super.onDestroy(); 
        if (mOpenCvCameraView != null)
            mOpenCvCameraView.disableView();
    }
    @Override
    public void onResume()
    {
        super.onResume();
        OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_3, this, mLoaderCallback);
    }

    int i =0 ;
    
	@Override
	public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
		mRgba = inputFrame.rgba();
        mGray = inputFrame.gray();

        prueba.update(inputFrame);
        
//		i++;
//		prueba.setA(i);

		System.out.println("iteracion:");
        
		return mRgba;
	}
	
    public native void FindFeatures(long matAddrGr, long matAddrRgba);

	@Override
	public boolean onTouch(View arg0, MotionEvent arg1) {

//		System.out.println(arg1.getX() + " " + arg1.getY());
		prueba.onTouchScreen();
		
		return false;
	}
       
}
