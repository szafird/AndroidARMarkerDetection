package com.dszafir.markerdetection;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Vector;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvException;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import android.util.Log;


/**
 * Marker detected in an image. It must be a four-squared contour with black border and
 * a valid code inside it. Modified version of Aruco Marker class by Rafael Ortega
 * 
 * @author Dan Szafir, Rafael Ortega
 */
public class Marker implements Comparable<Marker>{
	
	protected int id;
	protected float size;
	private int rotations;
	
	private Code code; // a matrix of integer representing the code (see the class to further explanation)
	
	private MatOfPoint2f mat; //the cvMat representing the camera capture of the marker
	private List<Point> points; //mat in list form
	
	private double perimeter;
	
	private Mat cononicalMat; // the cvMat of the CANONICAL marker (not the one taken from the capture)
	private Mat rVec; //rotation matrix
	private Mat tVec; //transformation matrix
	
	private Point center;
	
	private Mat cameraTransform;
	
	private Marker()
	{
		id = -1;
		code = new Code();
		rVec = new Mat(3,1,CvType.CV_64FC1);
		tVec = new Mat(3,1,CvType.CV_64FC1);
		cameraTransform = new Mat(4,4, CvType.CV_64F);
		cononicalMat = new Mat();
		mat = new MatOfPoint2f();
	}
	
	public Marker(float size, MatOfPoint2f data) {
		this();
		
		if (data.total() != 4) {
			throw new IllegalArgumentException("Error - markers can only have 4 points");
		}
		if (size <= 0) {
			throw new IllegalArgumentException("Error - marker size cannot be <= 0");
		}
		
		this.mat = data;
		this.points = mat.toList();
		this.size = size;
		
		calcCenter();
		calcPerimeter();
	}
	
	public Marker(float size, Vector<Point> p){
		this();
		this.size = size;
		points = p;
		this.mat.fromList(points);
		
		calcCenter();
		calcPerimeter();
	}
	
	public void draw(Mat in, CameraParameters cp, Scalar color, int lineWidth, boolean writeId) {
		
		//Set up the cube
		double halfSize = size/2.0;
		Vector<Point3> cubeVectorPoints = new Vector<Point3>();
		cubeVectorPoints.add(new Point3(-halfSize, -halfSize, 0));
		cubeVectorPoints.add(new Point3(-halfSize,  halfSize, 0));
		cubeVectorPoints.add(new Point3( halfSize,  halfSize, 0));
		cubeVectorPoints.add(new Point3( halfSize, -halfSize, 0));
		cubeVectorPoints.add(new Point3(-halfSize, -halfSize, size));
		cubeVectorPoints.add(new Point3(-halfSize,  halfSize, size));
		cubeVectorPoints.add(new Point3( halfSize,  halfSize, size));
		cubeVectorPoints.add(new Point3( halfSize, -halfSize, size));
		
		MatOfPoint3f objectPoints = new MatOfPoint3f();		
		objectPoints.fromList(cubeVectorPoints);
		MatOfPoint2f imagePoints = new MatOfPoint2f();
		
		//Project points into camera space
		Calib3d.projectPoints(objectPoints, rVec, tVec, cp.getCameraMatrix(), cp.getDistortionMatrixAsMat(), imagePoints);
		
		List<Point> pts = imagePoints.toList();
		
	    for (int i=0;i<4;i++){
	    	//draw the cube
	        Core.line(in ,pts.get(i),pts.get((i+1)%4), color, 2);
	        Core.line(in,pts.get(i+4),pts.get(4+(i+1)%4), color, 2);
	        Core.line(in,pts.get(i),pts.get(i+4), color, 2);
	        
	        //outline the marker
	        Core.line(in, points.get(i), points.get((i+1)%4), color, lineWidth);
	    }	        
		
	    if(writeId){
	    	String cad = "id="+Integer.toString(id);
	        Core.putText(in, cad, center, Core.FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
	    }
	    
	    draw3dAxis(in, cp);
	}
	
	public void drawOutline(Mat in, Scalar color, int lineWidth, boolean writeId){

	    for(int i=0;i<4;i++) {
	    	Core.line(in, points.get(i), points.get((i+1)%4), color, lineWidth);
//	    	Core.putText(in, Integer.toString(i), points.get(i), Core.FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
	    }
	    if(writeId){
	    	String cad = "id="+Integer.toString(id);
	        Core.putText(in, cad, center, Core.FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
	    }
	}
	
	public void draw3dCube(Mat frame, CameraParameters cp, Scalar color){
		MatOfPoint3f objectPoints = new MatOfPoint3f();
		double halfSize = size/2.0;
		Vector<Point3> points = new Vector<Point3>();
		points.add(new Point3(-halfSize, -halfSize, 0));
		points.add(new Point3(-halfSize,  halfSize, 0));
		points.add(new Point3( halfSize,  halfSize, 0));
		points.add(new Point3( halfSize, -halfSize, 0));
		points.add(new Point3(-halfSize, -halfSize, size));
		points.add(new Point3(-halfSize,  halfSize, size));
		points.add(new Point3( halfSize,  halfSize, size));
		points.add(new Point3( halfSize, -halfSize, size));
		
		objectPoints.fromList(points);
		MatOfPoint2f imagePoints = new MatOfPoint2f();
		Calib3d.projectPoints(objectPoints, rVec, tVec, cp.getCameraMatrix(), cp.getDistortionMatrixAsMat(), imagePoints);
		
		List<Point> pts = imagePoints.toList();
		// draw
	    for (int i=0;i<4;i++){
	        Core.line(frame ,pts.get(i),pts.get((i+1)%4), color, 2);
	        Core.line(frame,pts.get(i+4),pts.get(4+(i+1)%4), color, 2);
	        Core.line(frame,pts.get(i),pts.get(i+4), color, 2);
	    }	        
	}

	public void draw3dAxis(Mat frame, CameraParameters cp){
		MatOfPoint3f objectPoints = new MatOfPoint3f();
		Vector<Point3> points = new Vector<Point3>();
		
		points.add(new Point3(0, 0, 0));
		points.add(new Point3(size, 0, 0));
		points.add(new Point3(0, size, 0));
		points.add(new Point3(0, 0, size));
		
		objectPoints.fromList(points);
		MatOfPoint2f imagePoints = new MatOfPoint2f();
		Calib3d.projectPoints(objectPoints, rVec, tVec, cp.getCameraMatrix(), cp.getDistortionMatrixAsMat(), imagePoints);
		
		List<Point> pts = imagePoints.toList();
		// draw
		Core.line(frame, pts.get(0), pts.get(1), new Scalar(0,0,255), 2);
		Core.line(frame, pts.get(0), pts.get(2), new Scalar(0,255,0), 2);
		Core.line(frame, pts.get(0), pts.get(3), new Scalar(255,0,0), 2);
		Core.putText(frame, "x", pts.get(1), Core.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0,0,255), 2);
		Core.putText(frame, "y", pts.get(2), Core.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0,255,0), 2);
		Core.putText(frame, "z", pts.get(3), Core.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255,0,0), 2);	
	}

	private void calcPerimeter()
	{
		perimeter = 0;
		for(int i=0; i<4; ++i){
			perimeter += pointDistance(points.get(i), points.get((i+1)%4));
		}
	}

	/**
	 * Returns the perimeter of the marker, the addition of the distances between
	 * consecutive points.
	 * @return the perimeter.
	 */
	public double perimeter(){
		return perimeter;
	}
	
	private void calcCenter()
	{
			setPointsCounterClockwise();
			
			center = new Point(0,0);
			
			for(int i=0;i<4;i++){
				center.x += points.get(i).x;
				center.y += points.get(i).y;
	    	}
			center.x/=4.;
			center.y/=4.;
	}

	/**
	 * method to access the id, this only returns the id. Doesn't calculate it.
	 * @return the marker id.
	 */
	public int getMarkerId(){
		return id;
	}
	
	public Mat getMat() {
		return mat;
	}
	
	public static Mat createMarkerImage(int id,int size) throws CvException	{
	    if (id>=1024)
	    	throw new CvException("id out of range");
	    Mat marker = new Mat(size,size, CvType.CV_8UC1, new Scalar(0));
	    //for each line, create
	    int swidth=size/7;
	    int ids[]={0x10,0x17,0x09,0x0e};
	    for (int y=0;y<5;y++) {
	        int index=(id>>2*(4-y)) & 0x0003;
	        int val=ids[index];
	        for (int x=0;x<5;x++) {
	            Mat roi=marker.submat((x+1)*swidth, (x+2)*swidth,(y+1)*swidth,(y+2)*swidth);
	            if ( (( val>>(4-x) ) & 0x0001) != 0 )
	            	roi.setTo(new Scalar(255));
	            else
	            	roi.setTo(new Scalar(0));
	        }
	    }
	    return marker;
	}
		
	protected void setCononicalMat(Mat in){
//		in.copyTo(cononicalMat);
		cononicalMat = in;
	}
	
	/**
	 * construct the matrix of integers from the mat stored.
	 */
	protected void extractCode() {
		
		int rows = cononicalMat.rows();
		int cols = cononicalMat.cols();
		assert(rows == cols);
		Mat grey = new Mat();
		// change the color space if necessary
		if(cononicalMat.type() == CvType.CV_8UC1)
			grey = cononicalMat;
		else
			Imgproc.cvtColor(cononicalMat, grey, Imgproc.COLOR_RGBA2GRAY);
		// apply a threshold
		Imgproc.threshold(grey, grey, 125, 255, Imgproc.THRESH_BINARY|Imgproc.THRESH_OTSU);
		// the swidth is the width of each row
		int swidth = rows/7;
		// we go through all the rows
		for(int y=0;y<7;++y){
			for(int x=0;x<7;++x){
				int Xstart = x*swidth;
				int Ystart = y*swidth;
				Mat square = grey.submat(Xstart, Xstart+swidth, Ystart, Ystart+swidth);
				int nZ = Core.countNonZero(square);
				if(nZ > (swidth*swidth)/2)
					code.set(x, y, 1);
				else
					code.set(x,y,0);
			}
		}
	}
	
	/**
	 * Return the id read in the code inside a marker. Each marker is divided into 7x7 regions
	 * of which the inner 5x5 contain info, the border should always be black. This function
	 * assumes that the code has been extracted previously.
	 * @param in a marker
	 * @return the id of the marker
	 */
	protected int calculateMarkerId(){
		// check all the rotations of code
		Code[] rotations = new Code[4];
		rotations[0] = code;
		int[] dists = new int[4];
		dists[0] = hammDist(rotations[0]);
		int[] minDist = {dists[0],0};
		for(int i=1;i<4;i++){
			// rotate
			rotations[i] = Code.rotate(rotations[i-1]);
			dists[i] = hammDist(rotations[i]);
			if(dists[i] < minDist[0]){
				minDist[0] = dists[i];
				minDist[1] = i;
			}
		}
		this.rotations = minDist[1];
		if(minDist[0] != 0){
			return -1; // matching id not found
		}
		else{
			this.id = mat2id(rotations[minDist[1]]);
		}
		return id;
	}
	
	/**
	 * this functions checks if the whole border of the marker is black
	 * @return true if the border is black, false otherwise
	 */
	protected boolean checkBorder(){
		for(int i=0;i<7;i++){
			// normally we'll only check first and last square
			int inc = 6;
			if(i==0 || i==6)// in first and last row the whole row must be checked
				inc = 1;
			for(int j=0;j<7;j+=inc)
				if(code.get(i, j)==1)
					return false;
		}
		return true;
	}
	
	/**
	 * Calculate translation and rotation matrix for this marker.
	 * 
	 * @param camMatrix
	 * @param distCoeff
	 */
	protected void calculateExtrinsics(Mat camMatrix, MatOfDouble distCoeffs, float sizeMeters) {
		// set the obj 3D points
		double halfSize = sizeMeters/2.0;
		List<Point3> objPoints = new ArrayList<Point3>();
		objPoints.add(new Point3(-halfSize, -halfSize,0));
		objPoints.add(new Point3(-halfSize,  halfSize,0));
		objPoints.add(new Point3( halfSize,  halfSize,0));
		objPoints.add(new Point3( halfSize, -halfSize,0));

		MatOfPoint3f objPointsMat = new MatOfPoint3f();
		objPointsMat.fromList(objPoints);
//		Calib3d.solvePnP(objPointsMat, this, camMatrix, distCoeffs, Rvec, Tvec);
		
		Calib3d.solvePnP(objPointsMat, this.mat, camMatrix, distCoeffs, rVec, tVec, false, Calib3d.ITERATIVE);
		
//		Calib3d.solvePnPRansac(objPointsMat, this, camMatrix, distCoeffs, Rvec, Tvec); //false, 100, 1.5, minInliersCount, inliers, flags)
		
//		Utils.rotateXAxis(Rvec);
		
//		Calculate marker position with respect to camera
		
		rVec.convertTo(rVec, CvType.CV_32F);
		tVec.convertTo(tVec, CvType.CV_32F);
		
		Mat rotMat = new Mat(3, 3, CvType.CV_32F);
		
		Calib3d.Rodrigues(rVec, rotMat);
		
		double[][] transform = {{rotMat.get(0, 0)[0], rotMat.get(1, 0)[0], rotMat.get(2, 0)[0], tVec.get(0, 0)[0]},
				{rotMat.get(0, 1)[0], rotMat.get(1, 1)[0], rotMat.get(2, 1)[0], tVec.get(1, 0)[0]},
				{rotMat.get(0, 2)[0], rotMat.get(1, 2)[0], rotMat.get(2, 2)[0], tVec.get(2, 0)[0]}};
		
		cameraTransform.put(0, 0, transform[0]);
		cameraTransform.put(1, 0, transform[1]);
		cameraTransform.put(2, 0, transform[2]);
		
		cameraTransform = cameraTransform.inv();
		
//		double[] data = new double[4];
//		
//		for (int row = 0; row < 3; ++row) {
//			for (int col = 0; col < 3; ++col) {
//				data[col] = rotMat.get(row, col)[0]; // Copy rotation component
//			}
//			data[3] = Tvec.get(row, 0)[0]; // Copy translation component
//			cameraTransform.put(row, 0, data);
//		}
//		
//		// Since solvePnP finds camera location, w.r.t to marker pose, to get marker pose w.r.t to the camera we invert it.
//		cameraTransform = cameraTransform.inv();
//		
//		Mat rotMat = new Mat();
//		Calib3d.Rodrigues(Rvec, rotMat);
		
//		
	
//		rotMat = rotMat.t();
//		Tvec = Rvec.inv().mul(Tvec);
//		
//		rotMat = rotMat.col(0);
//		Mat m1 = rotMat.t();
//		m1 = m1.inv();
//		
//		Log.d("ADebugTag", m1.size().toString());
//		Log.d("ADebugTag", Tvec.size().toString());
//		
//		m1.mul(Tvec);
////		
//		distFromCamera = rotMat.t().inv().mul(Tvec); //-np.matrix(rotM).T * np.matrix(tvec);
	}
	
	public Mat getCameraTransform()
	{
		return cameraTransform;
	}
	
	public Mat getTVect()
	{
		return tVec;
	}
	
	protected void setPoints(List<Point> p){
		this.mat.fromList(p);
		this.points = p;
		
		calcCenter();
		calcPerimeter();
	}

	private int hammDist(Code code){
		int ids[][] = {
				{1,0,0,0,0},
				{1,0,1,1,1},
				{0,1,0,0,1},
				{0,1,1,1,0}
		};
		int dist = 0;
		for(int y=0;y<5;y++){
			int minSum = Integer.MAX_VALUE;
			// hamming distance to each possible word
			for(int p=0;p<4;p++){
				int sum=0;
				for(int x=0;x<5;x++)
					sum+= code.get(y+1,x+1) == ids[p][x]? 0:1;
				minSum = sum<minSum? sum:minSum;
			}
			dist+=minSum;
		}
		return dist;
	}

	private int mat2id(Code code){
		int val=0;
		for(int y=1;y<6;y++){
			val<<=1;
			if(code.get(y,2) == 1)
				val |= 1;
			val<<=1;
			if(code.get(y,4) == 1)
				val |= 1;
		}
		return val;
	}
	
	public int getRotations(){
		return this.rotations;
	}

	@Override
	public int compareTo(Marker other) {
		return id - other.id;
	}
	
	@Override
	public boolean equals(Object o) {
		if (!(o instanceof Marker)) return false;
		
		Marker other = (Marker)o;
		
		return this.id == other.id;
	}
	
	public void setPointsCounterClockwise()
	{
        // trace a line between the first and second point.
        // if the third point is at the right side, then the points are counter-clockwise
		double dx1 = points.get(1).x - points.get(0).x;
		double dy1 = points.get(1).y - points.get(0).y;
		double dx2 = points.get(2).x - points.get(0).x;
		double dy2 = points.get(2).y - points.get(0).y;
		double o = dx1*dy2 - dy1*dx2;
		if(o < 0.0){ // the third point is in the left side, we have to swap
			Collections.swap(points, 1, 3);
		}
		
		mat.fromList(points);
	}
	
	/**
	 * Calculate average distance from one marker to another
	 * @param m1
	 * @param m2
	 * @return
	 */
	public static double distance(Marker m1, Marker m2)
	{
		if (m1 == null || m2 == null) throw new IllegalArgumentException("Error: null input");
		
		if (m1.center == null) m1.calcCenter();
		if (m2.center == null) m2.calcCenter();
		
		return Math.sqrt(Math.pow(m1.center.x - m2.center.x,2) + 
				Math.pow(m1.center.y - m2.center.y, 2));
		
		
//		dist+=Math.sqrt(Math.pow(m1.points.get(0).x - m2.points.get(0).x,2) +
//				Math.pow(m1.points.get(0).y - m2.points.get(0).y,2));
//
//		dist+=Math.sqrt((fromPoints.get(1).x-toPoints.get(1).x)*(fromPoints.get(1).x-toPoints.get(1).x)+
//				(fromPoints.get(1).y-toPoints.get(1).y)*(fromPoints.get(1).y-toPoints.get(1).y));
//		
//		dist+=Math.sqrt((fromPoints.get(2).x-toPoints.get(2).x)*(fromPoints.get(2).x-toPoints.get(2).x)+
//				(fromPoints.get(2).y-toPoints.get(2).y)*(fromPoints.get(2).y-toPoints.get(2).y));
//		
//		dist+=Math.sqrt((fromPoints.get(3).x-toPoints.get(3).x)*(fromPoints.get(3).x-toPoints.get(3).x)+
//				(fromPoints.get(3).y-toPoints.get(3).y)*(fromPoints.get(3).y-toPoints.get(3).y));
//		
//		return dist/4;
	}
	
	public Point getPoint(int i) {
		return points.get(i);
	}
	
	public List<Point> getPoints() {
		return points;
	}
	
	public float getSize(){
		return size;
	}
	
	public static double pointDistance(Point p1, Point p2)
	{
		return Math.sqrt(Math.pow(p1.x-p2.x,2) + Math.pow(p1.y-p2.y, 2));
	}
}
