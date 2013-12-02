package com.dszafir.markerdetection;

import java.util.Collections;
import java.util.List;
import java.util.Vector;

import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.imgproc.Imgproc;

import android.util.Log;

/**
 * Marker Detector
 * 
 * Class to detect markers. Based on Aruco MarkerDetector class by Rafael Ortega
 * and http://image2measure.net/files/Mastering_OpenCV.pdf
 * 
 * Thresholds image, analyzes contours, and looks for valid marker codes.
 * @author Dan Szafir
 */
// TODO eliminate innecessary native calls, for example store the frame info 
// such as type in member fields and call it only once
public class MarkerDetector {
	
	private enum ThresholdMethod {FIXED_THRES,ADPT_THRES,CANNY};
		
	private float markerSizeMeters;
	private Size canonicalMarkerSize;
	private Mat markerCorners2d;
	private CameraParameters cameraParameters;
	
	private final static double MIN_DISTANCE = 10;
	
	public MarkerDetector(CameraParameters cameraParameters, float markerSizeMeters){
		
		if (cameraParameters == null || !cameraParameters.isValid())
			throw new IllegalArgumentException("Error - invalid camera parameters");
		
		if (markerSizeMeters <= 0) throw new IllegalArgumentException("Error - invalid marker size");
		
		this.cameraParameters = cameraParameters;
		this.markerSizeMeters = markerSizeMeters;
		
		canonicalMarkerSize = new Size(50,50);
//		canonicalMarkerSize = new Size(100,100);
		markerCorners2d = new Mat(4,1,CvType.CV_32FC2);
		markerCorners2d.put(0,0, 0,0,
				canonicalMarkerSize.width-1,0,
				canonicalMarkerSize.width-1,canonicalMarkerSize.height-1,
				   0,canonicalMarkerSize.height-1);
	}
	
	/**
	 * Detect markers in a CV Frame
	 * @param frame
	 * @return A list of detected markers
	 */
	public Vector<Marker> processFrame(CvCameraViewFrame frame)
	{
		return findMarkers(frame.gray());
	}
	
	/**
	 * Detect markers in a given Mat
	 * @param mat Matrix to detect markers
	 * @param isGrey Flag indicating if mat is greyscale or not
	 * @return A list of detected markers
	 */
	public Vector<Marker> processFrame(Mat mat, boolean isGrey)
	{
		if (isGrey)
		{
			return findMarkers(mat);
		}
		else
		{
			Mat gray = new Mat();
			Imgproc.cvtColor(mat, gray, Imgproc.COLOR_RGBA2GRAY);
			return findMarkers(gray);
		}
	}
	
	/**
	 * Detect markers given a Mat which must be greyscale
	 * @param grayImage
	 * @return A List of detected markers
	 */
	private Vector<Marker> findMarkers(Mat grayImage)
	{
		// Make it binary
	    Mat thresholdImg = performThreshold(grayImage, ThresholdMethod.ADPT_THRES);
	    
	    //Detect Contours
	    Vector<MatOfPoint> contours = findContours(thresholdImg, grayImage.cols() / 5);
	    
//	    Log.d("ADebugTag", "Contours: " + contours.size());
	    
	    // Find closed contours that can be approximated with 4 points
	    Vector<Marker> detectedMarkers = findCandidates(contours);
	    
//	    Log.d("ADebugTag", "Detected Markers: " + detectedMarkers.size());
	    
	    // Decode markers and ensure each is detected only once
	    Vector<Marker> visibleMarkers = recognizeMarkers(grayImage, detectedMarkers);
	    
//	    Log.d("ADebugTag", "Visible Markers: " + visibleMarkers.size());
	    
	    // Calculate their poses
	    estimatePosition(visibleMarkers);
	    
	    return visibleMarkers;
	}
	
	private Mat performThreshold(Mat src, ThresholdMethod method){
		Mat dst = new Mat();
		switch(method){
		case FIXED_THRES:
			Imgproc.threshold(src, dst, 127, 255, Imgproc.THRESH_BINARY_INV);
			break;
		case ADPT_THRES:
			Imgproc.adaptiveThreshold(src,dst,255.0,Imgproc.ADAPTIVE_THRESH_GAUSSIAN_C,
					Imgproc.THRESH_BINARY_INV, 7, 7); //(int)thresParam1,thresParam2);
			break;
		case CANNY:
			Imgproc.Canny(src, dst, 10, 220);// TODO this parameters??
			break;
		}
		return dst;
	}

	private Vector<MatOfPoint> findContours(Mat thresholdImg, int minContourPointsAllowed) {
		
		Vector<MatOfPoint> allContours = new Vector<MatOfPoint>();
		Mat hierarchy = new Mat();
		Imgproc.findContours(thresholdImg, allContours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_NONE);
	
		//remove all contours that are too small to be candidates
		for (int i = 0; i < allContours.size(); ++i) {
			if (allContours.get(i).total() < minContourPointsAllowed)
			{
				allContours.remove(i);
				--i;
			}
		}
		return allContours;
	}

	private Vector<Marker> findCandidates(Vector<MatOfPoint> contours) {
		
		Vector<Marker> detectedMarkers = new Vector<Marker>();
		
		MatOfPoint2f approxCurve = new MatOfPoint2f();
		for(int i=0;i<contours.size();i++){
			
			// Approximate to a polygon
			MatOfPoint2f contour = new MatOfPoint2f();
			contours.get(i).convertTo(contour, CvType.CV_32FC2);
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
	        
	        if (minDist < MIN_DISTANCE)
	            continue;
	        
	        // All tests are passed. Save marker candidate:
//	        Marker m = new Marker(markerSizeMeters, p);
//	        detectedMarkers.add(m);
	        detectedMarkers.add(new Marker(markerSizeMeters, p));//approxCurve));
		}
		
		//Remove elements whose corners are too close
		Marker m1, m2;
		for(int i = 0; i < detectedMarkers.size(); ++i) {
			for (int j = i+1; j < detectedMarkers.size(); ++j) {
				m1 = detectedMarkers.get(i);
				m2 = detectedMarkers.get(j);
	
				if (Marker.distance(m1, m2) < 10) {
					if(m1.perimeter()<m2.perimeter()) {
						//m1 is bad
						detectedMarkers.remove(i);
						j = detectedMarkers.size();
						--i;
					} else {
						//m2 is bad
						detectedMarkers.remove(j);
						--j;
					}
				}
			}
		}
		
		return detectedMarkers;
	}

	private Vector<Marker> recognizeMarkers(Mat grayImage, Vector<Marker> detectedMarkers) {
		
		Vector<Marker> visibleMarkers = new Vector<Marker>();
		
		for(int i = 0; i < detectedMarkers.size(); ++i) {
			Marker marker = detectedMarkers.get(i);
			
			// Find the perspective transformation that brings current marker to rectangular form
	        Mat markerTransform = Imgproc.getPerspectiveTransform(marker.getMat(), markerCorners2d);
	        
	        // Transform image to get a canonical marker image
	        Mat canonicalMarker = new Mat();
	        Imgproc.warpPerspective(grayImage, canonicalMarker,  markerTransform, canonicalMarkerSize);
	        
	        marker.setCononicalMat(canonicalMarker);
			marker.extractCode();
			
			if(marker.checkBorder()){
				int id = marker.calculateMarkerId();
				if (id == -1) {
					detectedMarkers.remove(i);
					i--;
					continue;
				}
				else {
					int index = visibleMarkers.indexOf(marker);
					if (index == -1) {
						// rotate the points of the marker so they are always in the same order no matter the camera orientation
						Collections.rotate(marker.getPoints(), 4-marker.getRotations());
						visibleMarkers.add(marker);
					}
					else {
						if (marker.perimeter() > visibleMarkers.get(index).perimeter()) {
							visibleMarkers.set(index, marker);
						}
					}
				}
			}
		}
		
		// Refine marker corners using sub pixel accuracy (not sure how necessary this is)
		if (visibleMarkers.size() > 0) {	
			
			List<Point> preciseCorners = new Vector<Point>();
			
			for (int i = 0; i < visibleMarkers.size(); ++i) {
				Marker marker = visibleMarkers.get(i);
				preciseCorners.addAll(marker.getPoints());
			}
			
			MatOfPoint2f prec = new MatOfPoint2f();
			prec.fromList(preciseCorners);
			
			TermCriteria termCriteria = new TermCriteria(TermCriteria.MAX_ITER | TermCriteria.EPS, 
					30, 0.01);
			Imgproc.cornerSubPix(grayImage, prec, new Size(5,5), new Size(-1,-1), termCriteria);
			
			preciseCorners = prec.toList();
			
			//Copy refined corners position back to markers
			for (int i = 0; i < visibleMarkers.size(); ++i) {
				Marker marker = visibleMarkers.get(i);
				marker.setPoints(preciseCorners.subList(i*4, i*4+4));
			}
		}
		
		return visibleMarkers;
	}

	private void estimatePosition(Vector<Marker> visibleMarkers) {
		
		//TODO: check if this is right...
		for (Marker marker : visibleMarkers) {
			marker.calculateExtrinsics(cameraParameters.getCameraMatrix(), 
					cameraParameters.getDistCoeff(), markerSizeMeters);
		}
	}
}
