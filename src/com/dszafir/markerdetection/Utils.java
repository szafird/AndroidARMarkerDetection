package com.dszafir.markerdetection;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.imgproc.Imgproc;

/**
 * Class containing utility functions.
 * 
 * @author Dan Szafir
 *
 */
public class Utils {

	/**
	 * Remove distortion caused by a fisheye lense using FOV model 
	 * (http://hal.inria.fr/docs/00/26/72/47/PDF/distcalib.pdf)
	 * 
	 * @param frame The input image
	 * @return The undistorted image
	 */
	public static Mat undistortFrame(Mat frame, FisheyeCameraParameters cp) {

		double Fx = cp.getFx();
		double Fy = cp.getFy();
		double Cx = cp.getCx(); 
		double Cy = cp.getCy();

		double w = cp.getW();
		double invW = 1/w;
		double theta = 2*Math.tan(w/2);

		double[] xMapArray = new double[frame.rows()*frame.cols()];
		double[] yMapArray = new double[frame.rows()*frame.cols()];
		for( int j = 0; j < frame.rows(); j++ )
		{
			for( int i = 0; i < frame.cols(); i++ )
			{
				// U & V are real would units from the optical center. Defined by Cx and Cy (in pixels)
				double u = (i - Cx) / Fx;
				double v = (j - Cy) / Fy;

				// Calulate undistorted vector norm
				double Ru = Math.sqrt(u*u + v*v);

				// Calculate distorted vector norm
				double Rd = invW * Math.atan(Ru * theta);

				// Apply the magnitude change and convert back to pixel units
				double xVal =  Rd * u * Fx / Ru + Cx;
				double yVal =  Rd * v * Fy / Ru + Cy;

				xMapArray[j*(frame.cols()) + i] = xVal;
				yMapArray[j*(frame.cols()) + i] = yVal;
			}
		}

		Mat xMapMat = new Mat(frame.size(), CvType.CV_32FC1);
		Mat yMapMat = new Mat(frame.size(), CvType.CV_32FC1);
		xMapMat.put(0,0,xMapArray);
		yMapMat.put(0,0,yMapArray);

		Mat undistorted = new Mat();
		Imgproc.remap(frame, undistorted, xMapMat, yMapMat, Imgproc.INTER_LINEAR);

		return undistorted;
	}

	public static MatOfPoint2f undistortFrame(MatOfPoint2f frame, FisheyeCameraParameters cp) {

		double Fx = cp.getFx();
		double Fy = cp.getFy();
		double Cx = cp.getCx(); 
		double Cy = cp.getCy();

		double w = cp.getW();
		double invW = 1/w;
		double theta = 2*Math.tan(w/2);

		double[] xMapArray = new double[frame.rows()*frame.cols()];
		double[] yMapArray = new double[frame.rows()*frame.cols()];
		for( int j = 0; j < frame.rows(); j++ )
		{
			for( int i = 0; i < frame.cols(); i++ )
			{
				// U & V are real would units from the optical center. Defined by Cu and Cv (in pixels)
				double u = (i - Cx) / Fx;
				double v = (j - Cy) / Fy;

				// Calulate undistorted vector norm
				double Ru = Math.sqrt(u*u + v*v);

				// Calculate distorted vector norm
				double Rd = invW * Math.atan(Ru * theta);

				// Apply the magnitude change and convert back to pixel units
				double xVal =  Rd * u * Fx / Ru + Cx;
				double yVal =  Rd * v * Fy / Ru + Cy;

				xMapArray[j*(frame.cols()) + i] = xVal;
				yMapArray[j*(frame.cols()) + i] = yVal;
			}
		}

		Mat xMapMat = new Mat(frame.size(), CvType.CV_32FC1);
		Mat yMapMat = new Mat(frame.size(), CvType.CV_32FC1);
		xMapMat.put(0,0,xMapArray);
		yMapMat.put(0,0,yMapArray);

		MatOfPoint2f undistorted = new MatOfPoint2f();
		Imgproc.remap(frame, undistorted, xMapMat, yMapMat, Imgproc.INTER_LINEAR);

		return undistorted;
	}
}
