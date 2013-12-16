package com.dszafir.markerdetection;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
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
	public static Mat undistortFrame(Mat frame) {
		
		Mat mapx = new Mat(frame.size(), CvType.CV_32FC1);
		Mat mapy = new Mat(frame.size(), CvType.CV_32FC1);

		//Camera focal lengths in pixels
		double fu = 258.83;
		double fv = 258.346;   
 
		//Optical center (pixels)
		double Cu = 325.292;   
		double Cv = 247.001;
		
		//Field of view
		double w = 0.925107;
		
		double invW = 1/w;
		double theta = Math.tan(w/2);
		
		double[][] xs = new double[frame.rows()][frame.cols()];
		double[][] ys = new double[frame.rows()][frame.cols()];
		for( int j = 0; j < frame.rows(); j++ )
		{
			for( int i = 0; i < frame.cols(); i++ )
			{	
					// U & V are real would units from the optical center. Defined by Cu and Cv (in pixels)
					double u = (i - Cu) / fu;
					double v = (j - Cv) / fv;
					
					// Calculate undistorted vector norm
					double ru = Math.sqrt(u*u + v*v);

					// Calculate distorted vector norm
					double rd = invW * Math.atan(2 * ru * theta);
					
					// Apply the magnitude change and convert back to pixel units
					double xVal =  rd * u * fu / ru + Cu;
					double yVal =  rd * v * fv / ru + Cv;
					
					xs[j][i] = xVal;					
					ys[j][i] = yVal;
		    }
		}
		
		for(int row = 0; row < xs.length; ++row) {
			mapx.put(row, 0, xs[row]);
			mapy.put(row, 0, ys[row]);
		}
		
		Mat undistorted = new Mat();
		Imgproc.remap(frame, undistorted, mapx, mapy, Imgproc.INTER_LINEAR);
		
		return undistorted;
	}
}
