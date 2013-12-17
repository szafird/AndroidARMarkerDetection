package com.dszafir.markerdetection;

import java.io.File;
import java.io.FilenameFilter;
import java.io.IOException;
import java.util.Vector;
import java.util.concurrent.ScheduledExecutorService;

import javax.xml.parsers.ParserConfigurationException;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.xml.sax.SAXException;

import android.app.Activity;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.SurfaceView;
import android.view.WindowManager;

/**
 * The main activity. This attempts to initialize and set up the camera using OpenCV4Android
 * and passes each captured frame to the MarkerDetector for processing.
 * 
 * @author Dan Szafir
 */
public class DetectMarkerActivity extends Activity implements CvCameraViewListener2{
	
	private enum Model {PINHOLE, FOV};
	
	private Model model = Model.PINHOLE;
	
	private MenuItem modelMenuItem;

	private static final String TAG = "MarkerDetectionDebug::Activity";
	
	private ScheduledExecutorService scheduleDetectTaskExecutor;

	private CameraBridgeViewBase mOpenCvCameraView;
	
	private MarkerDetector detector;
	
	private Vector<Marker> detectedMarkers;
	
	private CameraParameters pinholeCameraParameters;
	private FisheyeCameraParameters fisheyeCameraParameters;
	private CameraParameters currentCameraParameters;
	
	private float markerSizeMeters;
	
	private String path;
	
	private PGMImage[] pgmImages;
	private int pgmImageCounter;

	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
		setContentView(R.layout.activity_detect_marker);
		
		//Init camera view
		mOpenCvCameraView = (CameraBridgeViewBase) findViewById(R.id.MarkerCameraView);
		mOpenCvCameraView.setVisibility(SurfaceView.VISIBLE);
		mOpenCvCameraView.setCvCameraViewListener(this);
		
		path = Environment.getExternalStorageDirectory().getAbsolutePath()+"/calibration/";
	}
	
	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		
		modelMenuItem = menu.add("FOV Model (Pre-recorded)");
		
		// Inflate the menu; this adds items to the action bar if it is present.
		getMenuInflater().inflate(R.menu.detect_marker, menu);
		
		return true;
	}
	
	@Override
	public boolean onOptionsItemSelected(MenuItem item) {
		if (item == modelMenuItem) {
			if (model == Model.PINHOLE)
			{
				currentCameraParameters = fisheyeCameraParameters;
				model = Model.FOV;
				modelMenuItem.setTitle("Pinhole (Live)");
			}
			else if (model == Model.FOV) {
				currentCameraParameters = pinholeCameraParameters;
				model = Model.PINHOLE;
				modelMenuItem.setTitle("FOV Model (Pre-recorded)");
			}
			detector.setCameraParameters(currentCameraParameters);
		}
		return true;
	}
	
	/**
	 * Init parameters for marker detection. Should only be called after OpenCV initialization
	 * callback has completed.
	 */
	private void init()
	{
		File pgmDir = new File(path + "/pgms/");
		if (pgmDir.exists() && pgmDir.isDirectory()) {
			
			File[] pgmFiles = pgmDir.listFiles(new FilenameFilter() {
				
				@Override
				public boolean accept(File dir, String name) {
					return name.endsWith(".pgm");
				}
			});
		
			pgmImages = new PGMImage[pgmFiles.length];
			for (int i = 0; i < pgmFiles.length; ++i) {
				try {
					pgmImages[i] = new PGMImage(pgmFiles[i]);
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		}
		
		detectedMarkers = new Vector<Marker>();
		
		try {
			pinholeCameraParameters = new CameraParameters(path+"camera.xml");
		} catch (ParserConfigurationException e) {
			e.printStackTrace();
		} catch (SAXException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}

		fisheyeCameraParameters = new FisheyeCameraParameters(258.83, 258.346, 325.292, 247.001, 0.925107);
		
		currentCameraParameters = pinholeCameraParameters;
		
//		markerSizeMeters = 0.0238f;
//		markerSizeMeters = 0.0475f; //half the size of the marker in meters
//		markerSizeMeters = 0.034f;
		markerSizeMeters = 0.095f; //marker size in meters
		
		detector = new MarkerDetector(pinholeCameraParameters, markerSizeMeters);
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
		Mat greyMat = null;
		if (model == Model.PINHOLE)
		{
			greyMat = inputFrame.gray();
		}
		else if (model == Model.FOV	)
		{
			greyMat = pgmImages[pgmImageCounter].getCameraImage();
			greyMat = Utils.undistortFrame(greyMat, fisheyeCameraParameters);
		}
		
		detectedMarkers = detector.processFrame(greyMat, true);
		
		Mat rgbaMat = null;
		
		if (model == Model.PINHOLE)
		{
			rgbaMat = inputFrame.rgba();
		}
		else if (model == Model.FOV	)
		{
			rgbaMat = new Mat();
			Imgproc.cvtColor(greyMat, rgbaMat, Imgproc.COLOR_GRAY2RGBA);
		}
		
		for(Marker m : detectedMarkers) {
			m.draw(rgbaMat, currentCameraParameters, new Scalar(255,0,0), 3, true);
			
//			double x,y,z;
//			x = m.getTVect().get(0,0)[0];
//			y = m.getTVect().get(1,0)[0];
//			z = m.getTVect().get(2, 0)[0];
//			
//			double distance = Math.sqrt(Math.pow(x,2) + Math.pow(y,2) + Math.pow(z,2));
//			
//			Core.putText(frame, Double.toString(distance), new Point(frame.width()/2, frame.height()/2), 
//					Core.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255,255,255), 2);
		}
		
		if (model == Model.PINHOLE)
		{
			return rgbaMat;
		}
		else if (model == Model.FOV	)
		{
			++pgmImageCounter;
			if (pgmImageCounter == pgmImages.length) pgmImageCounter = 0;

			// My android phone requires a 1280x720 Mat to convert to a bitmap
			if (rgbaMat.cols() == 1280 && rgbaMat.rows() == 720) return rgbaMat;
			
			//  Resize works, but the drawn cube will appear distorted
			//	Imgproc.resize(rgbaFrame, rgbaFrame, new Size(1280,720));

			//Instead, fill in a black border to get the right size
			Mat outputFrame = new Mat(new Size(1280, 720), rgbaMat.type());
			
			//Calculate border sizes
			int left = (1280 - rgbaMat.cols())/2;
			int right = left;
			//Correct for potential non-divisible by 2 error
			if (left + right + rgbaMat.cols() != 1280) ++right;
			int top = (720 - rgbaMat.rows())/2;
			int bottom = top;
			//Correct for potential non-divisible by 2 error
			if (top + bottom + rgbaMat.rows() != 720) ++bottom;
			
			Imgproc.copyMakeBorder(rgbaMat, outputFrame, top, bottom, left, right, Imgproc.BORDER_CONSTANT, new Scalar(0,0,0));

			return outputFrame;
		}
		else
		{
			return inputFrame.rgba();
		}
	}
	
//	private Mat undistortFrame(Mat frame) {
//		double fu = 258.83f;
//		double fv = 258.346f;     
//		
//		double w = 0.925107;
//		double invW = 1/w;
//		double theta = 2*Math.tan(w/2);
//		
//		double Cu = 325.292;   
//		double Cv = 247.001;
//
//		double[] xMapArray = new double[frame.rows()*frame.cols()];
//		double[] yMapArray = new double[frame.rows()*frame.cols()];
//		for( int j = 0; j < frame.rows(); j++ )
//		{
//			for( int i = 0; i < frame.cols(); i++ )
//			{
//					// U & V are real would units from the optical center. Defined by Cu and Cv (in pixels)
//					double u = (i - Cu) / fu;
//					double v = (j - Cv) / fv;
//					
//					// Calulate undistorted vector norm
//					double ru = Math.sqrt(u*u + v*v);
//
//					// Calculate distorted vector norm
//					double rd = invW * Math.atan(ru * theta)/(w);
//					
//					// Apply the magnitude change and convert back to pixel units
//					double xVal =  rd * u * fu / ru + Cu;
//					double yVal =  rd * v * fv / ru + Cv;
//					
//					xMapArray[j*(frame.cols()) + i] = xVal;
//					yMapArray[j*(frame.cols()) + i] = yVal;
//		    }
//		}
//		
//		Mat xMapMat = new Mat(frame.size(), CvType.CV_32FC1);
//		Mat yMapMat = new Mat(frame.size(), CvType.CV_32FC1);
//		xMapMat.put(0,0,xMapArray);
//		yMapMat.put(0,0,yMapArray);
//		
//		Mat undistorted = new Mat();
//		Imgproc.remap(frame, undistorted, xMapMat, yMapMat, Imgproc.INTER_LINEAR);
//		
//		return undistorted;
//	}

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
