package frc.robot;

import java.util.*;

public class PurePursuit {

    double spacing = 6;
    double dt = 0.1;

    //smoothing
    double weight_smooth = .85; // larger = smoother, .75-.98
    double weight_data = 1-weight_smooth;
    double tolerance = 0.001;
    
    public void generatePath(double x, double y, double headingCorrection) {
        // get rough path of points
        double[][] rawPath = Robot.pathfinder.createPositionalPath(x, y, headingCorrection, dt);

        // inject additional points
        ArrayList<double[]> injectedPath_list = new ArrayList<>();
        for(int i = 0; i < rawPath.length-1; i++) {
            double[] start = rawPath[i];
            double[] end = rawPath[i+1];
            double[] vector = {end[0]-start[0], end[1]-start[1]};
            double vector_magnitude = Math.ceil(Math.sqrt(vector[0]*vector[0] + vector[1]*vector[1]));

            double[] vector_normalized = {vector[0]/vector_magnitude, vector[1]/vector_magnitude};

            int fittablePoints = (int) (vector_magnitude/spacing);

            for(int j = 0; j < fittablePoints; j++) {
                double[] newVector = {start[0]+vector_normalized[0]*spacing*i, start[1]+vector_normalized[1]*spacing*i};
                injectedPath_list.add(newVector);
            }
        }
        injectedPath_list.add(rawPath[rawPath.length-1]);

        Double[][] injectedPath_arr = injectedPath_list.toArray(new Double[injectedPath_list.size()][2]);
        double[][] injectedPath = new double[injectedPath_arr.length][2];
        for(int i = 0; i < injectedPath_arr.length; i++) {
            injectedPath[i][0] = injectedPath_arr[i][0];
            injectedPath[i][1] = injectedPath_arr[i][1];
        }

        // smooth path
        double[][] smoothedPath = smoother(injectedPath, weight_data, weight_smooth, tolerance);

        ArrayList<PPPoint> trajectory = new ArrayList<>();
        trajectory.add(new PPPoint(smoothedPath[0][0], smoothedPath[0][1], 0, 0));  // first point

        for(int i = 1; i < smoothedPath.length; i++) {
            double distance = trajectory.get(i-1).distance + distanceFormula(trajectory.get(0).x, trajectory.get(0).y, 
                                                                                smoothedPath[i][0], smoothedPath[i][0]);
            
        }
    }

    public double distanceFormula(double x1, double y1, double x2, double y2) {
        return Math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
    }

    // stolen from 2168
    public double[][] smoother(double[][] path, double weight_data, double weight_smooth, double tolerance)
	{

		//copy array
		double[][] newPath = doubleArrayCopy(path);

		double change = tolerance;
		while(change >= tolerance)
		{
			change = 0.0;
			for(int i=1; i<path.length-1; i++)
				for(int j=0; j<path[i].length; j++)
				{
					double aux = newPath[i][j];
					newPath[i][j] += weight_data * (path[i][j] - newPath[i][j]) + weight_smooth * (newPath[i-1][j] + newPath[i+1][j] - (2.0 * newPath[i][j]));
					change += Math.abs(aux - newPath[i][j]);	
				}					
		}

		return newPath;
	}

    public static double[][] doubleArrayCopy(double[][] arr)
	{

		//size first dimension of array
		double[][] temp = new double[arr.length][arr[0].length];

		for(int i=0; i<arr.length; i++)
		{
			//Resize second dimension of array
			temp[i] = new double[arr[i].length];

			//Copy Contents
			for(int j=0; j<arr[i].length; j++)
				temp[i][j] = arr[i][j];
		}

		return temp;

    }
}