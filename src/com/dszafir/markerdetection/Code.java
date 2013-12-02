package com.dszafir.markerdetection;

/**
 * The matrix of int representing the code content of a marker.
 * It will have 7x7 dimensions.
 * 0->black
 * 1->white
 * 
 * Modified version of Aruco Code class by Rafael Ortega.
 *
 *@author Dan Szafir, Rafael Ortega
 */
public class Code {
	protected int[][] code;
	
	protected Code(){
		code = new int[7][7];
	}
	
	protected void set(int x, int y, int value){
		checkParameters(x, y);
		
		code[x][y] = value;
	}

	protected int get(int x, int y){	
		checkParameters(x, y);
		
		return code[x][y];
	}
	
	private void checkParameters(int x, int y)
	{
		if (x < 0 || x >= code.length) 
			throw new IllegalArgumentException("Error: value " + x + " " +
					"is invalid for argument x (must be in range [0," + code.length + ")");
		
		if (y < 0 || y >= code[x].length) 
			throw new IllegalArgumentException("Error: value " + y + " " +
					"is invalid for argument y (must be in range [0," + code[x].length + ")");
	}
	
	public static Code rotate(Code in){
		Code out = new Code();
		for(int i=0;i<7;i++)
			for(int j=0;j<7;j++){
				out.code[i][j] = in.code[6-j][i];
			}
		return out;
	}
}
