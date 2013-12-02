package com.dszafir.markerdetection;

import java.io.IOException;
import java.util.Vector;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import javax.xml.parsers.ParserConfigurationException;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.xml.sax.SAXException;

import android.app.Activity;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.SurfaceView;
import android.view.WindowManager;

/**
 * The main activity. This attempts to initialize and set up the camera using OpenCV4Android
 * and passes each captured frame to the MarkerDetector for processing.
 * 
 * @author Dan Szafir
 */
public class DetectMarkerActivity extends Activity implements CvCameraViewListener2{

	private static final String TAG = "MarkerDetectionDebug::Activity";
	
	private ScheduledExecutorService scheduleDetectTaskExecutor;

	private CameraBridgeViewBase mOpenCvCameraView;
	
	private MarkerDetector detector;
	
	private Vector<Marker> detectedMarkers;
	private CameraParameters cameraParameters;
	private float markerSizeMeters;
	
	private Mat markerCorners2d;
	private Size canonicalMarkerSize;

	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
		setContentView(R.layout.activity_detect_marker);
		
		//Init camera view
		mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.MarkerCameraView);
		mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);
		mOpenCvCameraView.setCvCameraViewListener(this);
	}
	
	/**
	 * Init parameters for marker detection. Should only be called after OpenCV initialization
	 * callback has completed.
	 */
	private void init()
	{
		//Init detector and parameters
//		detector = new MarkerDetector();
		
		detectedMarkers = new Vector<Marker>();

		cameraParameters = new CameraParameters();
		try {
			cameraParameters.readFromXML(Environment.getExternalStorageDirectory().getAbsolutePath()+"/calibration/camera.xml");
		} catch (ParserConfigurationException e) {
			e.printStackTrace();
		} catch (SAXException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}

//		markerSizeMeters = 0.0238f;
//		markerSizeMeters = 0.0475f; //half the size of the marker in meters
//		markerSizeMeters = 0.034f;
		markerSizeMeters = 0.095f; //marker size in meters
		
		detector = new MarkerDetector(cameraParameters, markerSizeMeters);
		
		
		canonicalMarkerSize = new Size(50,50);
		markerCorners2d = new Mat(4,1,CvType.CV_32FC2);
		markerCorners2d.put(0,0, 0,0,
				canonicalMarkerSize.width-1,0,
				canonicalMarkerSize.width-1,canonicalMarkerSize.height-1,
				   0,canonicalMarkerSize.height-1);
	}

	@Override
	public void onResume()
	{
		super.onResume();
		OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_2_4_6, this, mLoaderCallback);
	}

	@Override
	public void onPause()
	{
		super.onPause();
		if (mOpenCvCameraView != null)
			mOpenCvCameraView.disableView();
	}

	public void onDestroy() {
		super.onDestroy();
		if (mOpenCvCameraView != null)
			mOpenCvCameraView.disableView();
	}

	public void onCameraViewStarted(int width, int height) {
	}

	public void onCameraViewStopped() {
	}

	/**
	 * Analyze each frame for markers
	 */
	public Mat onCameraFrame(CvCameraViewFrame inputFrame) {

		Mat frame = inputFrame.rgba();
		detectedMarkers = detector.processFrame(inputFrame);

		for(Marker m : detectedMarkers) {
			m.draw(frame, cameraParameters, new Scalar(255,0,0), 3, true);
			
			double x,y,z;
			x = m.getTVect().get(0,0)[0];
			y = m.getTVect().get(1,0)[0];
			z = m.getTVect().get(2, 0)[0];
			
			double distance = Math.sqrt(Math.pow(x,2) + Math.pow(y,2) + Math.pow(z,2));
			
			Core.putText(frame, Double.toString(distance), new Point(frame.width()/2, frame.height()/2), 
					Core.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255,255,255), 2);
		}
		
		
		
		return frame;
//		Mat undistort = new Mat();
//		Imgproc.undistort(frame, undistort, cameraParameters.getCameraMatrix(), cameraParameters.getDistCoeff());
//		
//		return undistort;
	}
	
	/**
	 * OpenCV Initialization Callback
	 */
	private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
		@Override
		public void onManagerConnected(int status) {
			switch (status) {
			case LoaderCallbackInterface.SUCCESS:
			{
				//Now can call OpenCV code
				Log.i(TAG, "OpenCV loaded successfully");
				//		                mOpenCvCameraView.enableView();
				//0 = rear cam, 1 = front cam
				//		                mOpenCvCameraView.setCameraIndex(1);
				mOpenCvCameraView.enableFpsMeter();
				mOpenCvCameraView.enableView();
				
				init();
			} break;
			default:
			{
				super.onManagerConnected(status);
			} break;
			}
		}
	};
}
