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
	
//	private OriginalMarkerDetector detector;
	private MarkerDetector detector;
	
	private Vector<Marker> detectedMarkers;
	private CameraParameters cameraParameters;
	private float markerSizeMeters;
	private boolean detectFlag;
	
	private Mat markerCorners2d;
	private Size canonicalMarkerSize;
	
	private Mat thresh, thresh2;

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
//		detector = new OriginalMarkerDetector();
		
		thresh = new Mat(); thresh2 = new Mat();
		
		canonicalMarkerSize = new Size(50,50);
		markerCorners2d = new Mat(4,1,CvType.CV_32FC2);
		markerCorners2d.put(0,0, 0,0,
				canonicalMarkerSize.width-1,0,
				canonicalMarkerSize.width-1,canonicalMarkerSize.height-1,
				   0,canonicalMarkerSize.height-1);
		
		scheduleDetectTaskExecutor = Executors.newScheduledThreadPool(5);
		scheduleDetectTaskExecutor.scheduleAtFixedRate(new Runnable() {
		      public void run() {
		    	  detectFlag = true;
		      }
		    }, 0, 500, TimeUnit.MILLISECONDS);
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
//		Vector<OriginalMarker> detectedMarkers = new Vector<OriginalMarker>();
//		detector.detect(frame, detectedMarkers, cameraParameters, markerSizeMeters, frame);
//		for(OriginalMarker m : detectedMarkers){
		for(Marker m : detectedMarkers) {
			m.draw(frame, cameraParameters, new Scalar(255,0,0), 3, true);
//			m.draw3dAxis(frame, cameraParameters);
//			m.draw3dCube(frame, cameraParameters, new Scalar(255,0,0));
			
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
	/*
//		Mat greyFrame = inputFrame.gray();
//		Mat newFrame = new Mat();
//		Imgproc.threshold(greyFrame, newFrame, 100, 255, Imgproc.THRESH_BINARY);
//		return newFrame;
		
		detectedMarkers.clear();
		
		Vector<Marker> visibleMarkers = new Vector<Marker>();
		
		Mat frame = inputFrame.rgba();
		Mat grayFrame = inputFrame.gray();
		
		//-------Threshold-------//
		
		Imgproc.adaptiveThreshold(grayFrame,thresh,255.0,Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C,
				Imgproc.THRESH_BINARY_INV,(int)7,7);

		//-------Find Contours-------//
		
		// pass a copy because it modifies the src image
		thresh.copyTo(thresh2);
		
		Mat hierarchy2 = new Mat();
		Vector<MatOfPoint> contours2 = new Vector<MatOfPoint>();
		Imgproc.findContours(thresh2, contours2, hierarchy2, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
		
		for (int i = 0; i < contours2.size(); ++i) {
			if (contours2.get(i).total() < grayFrame.cols()/5)
			{
				contours2.remove(i);
				--i;
			}
		}
		
//		Imgproc.drawContours(frame, contours2, -1, new Scalar(255,0,0),2);
		
		//-------Find Candidates-------//
		
		MatOfPoint2f approxCurve = new MatOfPoint2f();
		for(int i=0;i<contours2.size();i++){
			
			// Approximate to a polygon
			MatOfPoint2f contour = new MatOfPoint2f();
			contours2.get(i).convertTo(contour, CvType.CV_32FC2);
			double eps = contour.total() * .05;
			Imgproc.approxPolyDP(contour, approxCurve, eps, true);
			
			// We interested only in polygons that contains only four points
	        if (approxCurve.total() != 4)
	            continue;
			
	        // And they have to be convex
	        MatOfPoint mat = new MatOfPoint();
			approxCurve.convertTo(mat, CvType.CV_32SC2);
	        if (!Imgproc.isContourConvex(mat))
	            continue;
	        
	        // Ensure that the distance between consecutive points is large enough
	        double minDist = Double.MAX_VALUE;;

	        Vector<Point> p = new Vector<Point>();
	        float[] points = new float[8];// [x1 y1 x2 y2 x3 y3 x4 y4]
			approxCurve.get(0,0,points);
	        for (int j = 0; j < 8; j += 2)
	        {
	        	double d = Math.sqrt(Math.pow((points[j]-points[(j+2)%8]), 2) +
						Math.pow((points[j+1]-points[(j+3)%8]),2));
	            minDist = Math.min(minDist, d);
	            
	            p.add(new Point(points[j], points[j+1]));
	        }
	        
	        if (minDist < 10)
	            continue;
	        
	        // All tests are passed. Save marker candidate:
	        Marker m = new Marker(markerSizeMeters, p);
	        m.calcCenter();
	        detectedMarkers.add(new Marker(markerSizeMeters, p));
		}
		
		cleanup(detectedMarkers);
		
		//-------Recognize Markers-------//
		for(int i = 0; i < detectedMarkers.size(); ++i) {
			Marker m = detectedMarkers.get(i);
			
			// Find the perspective transformation that brings current marker to rectangular form
	        Mat markerTransform = Imgproc.getPerspectiveTransform(m, markerCorners2d);
	        
	        // Transform image to get a canonical marker image
	        Mat canonicalMarker = new Mat();
	        Imgproc.warpPerspective(grayFrame, canonicalMarker,  markerTransform, canonicalMarkerSize);
	        
	        m.setMat(canonicalMarker);
			m.extractCode();
			
			if(m.checkBorder()){
				int id = m.calculateMarkerId();
				if (id == -1) {
					detectedMarkers.remove(i);
					i--;
					continue;
				}
				else {
					int index = visibleMarkers.indexOf(m);
					if (index == -1) {
						// rotate the points of the marker so they are always in the same order no matter the camera orientation
						Collections.rotate(m.toList(), 4-m.getRotations());
						visibleMarkers.add(m);
					}
					else {
						if (m.perimeter() > visibleMarkers.get(index).perimeter()) {
							visibleMarkers.set(index, m);
						}
					}
				}
			}
		}
		
		for(Marker m : visibleMarkers){
//		if(m.getMarkerId() == idSelected){
//			m.draw3dCube(frame, super.mView.mCamParam, new Scalar(255,0,0));
//		}
//		else{
			if (cameraParameters.isValid()) {
				m.calculateExtrinsics(cameraParameters.getCameraMatrix(), 
						cameraParameters.getDistCoeff(), markerSizeMeters);
			}
//			m.draw(frame, new Scalar(255,0,0), 3, true);
			m.draw3dCube(frame, cameraParameters, new Scalar(255,0,0));
//		}
	}
		
		return frame;
		
//		
//		if (detectFlag) {
//			detectedMarkers = detector.detect(frame, grayFrame, cameraParameters, 
//					markerSize);
////			detectedMarker = detector.detectSingleMarker(frame, grayFrame, cameraParameters,
////					markerSize);
//			
//			detectFlag = false;
//		}
//		
////		if (detectedMarker != null) {
////			detectedMarker.draw(frame, new Scalar(255,0,0), 3, true);
////		}
//		for(Marker m : detectedMarkers){
////			if(m.getMarkerId() == idSelected){
////				m.draw3dCube(frame, super.mView.mCamParam, new Scalar(255,0,0));
////			}
////			else{
//				m.draw(frame, new Scalar(255,0,0), 3, true);
////			}
//		}
//
//		return frame;
	}
	
	private void cleanup(Vector<Marker> candidateMarkers)
	{
		//Remove elements whose corners are too close and ensure each marker is only detected once
		Marker m1, m2;
		boolean goodCandidate = false;
		for(int i = 0; i < candidateMarkers.size(); ++i) {
			goodCandidate = false;
			for (int j = i+1; j < candidateMarkers.size(); ++j) {
				m1 = candidateMarkers.get(i);
				m2 = candidateMarkers.get(j);

				goodCandidate = true;

				if (Marker.distance(m1, m2) < 10) {
					if(m1.perimeter()<m2.perimeter()) {
						//m1 is bad
						candidateMarkers.remove(i);
						j = candidateMarkers.size();
						--i;
						goodCandidate = false;
					} else {
						//m2 is bad
						candidateMarkers.remove(j);
						--j;
					}
				}
			}
		}
	}
*/
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
