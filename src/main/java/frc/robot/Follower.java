
package frc.robot;

import java.io.*;
import java.util.*;

public class Follower{

    /*
    ***List of Paths***
    LtoML: Starts from left side, goes to mid-left hatch
    */

    public double maxVelocity = 13;
    public double kv = 1/maxVelocity;
    public double ka = 0.025;
    public double kp = 0.003;
    public double kd = 0;

    public final double dt = 0.01;
    final String folder = "/home/lvuser/paths/";

    public TreeMap<String, ArrayList<ArrayList<TPoint>>> paths = new TreeMap<>(); // 0: left, 1:right

    ArrayList<TPoint> LtoML_l = new ArrayList<>();
    ArrayList<TPoint> LtoML_r = new ArrayList<>();


    public Follower() {
        try {
            readPath("LtoML");
            readPath("BackUp");
        }
        catch(Exception e) {
            e.printStackTrace();
        }
    }   

    public void readPath(String path) throws IOException{
        Scanner L_scanner = new Scanner(new File(folder + path + "_left.csv"));
        Scanner R_scanner = new Scanner(new File(folder + path + "_right.csv"));

        ArrayList<TPoint> list_l = new ArrayList<>();
        ArrayList<TPoint> list_r = new ArrayList<>();

        while(L_scanner.hasNext()) {
            String[] delimited_l = L_scanner.nextLine().split(",");
            String[] delimited_r = R_scanner.nextLine().split(",");
            TPoint left = new TPoint(Double.parseDouble(delimited_l[0]), Double.parseDouble(delimited_l[1]), 
                                     Double.parseDouble(delimited_l[2]), Math.toRadians(Double.parseDouble(delimited_l[3])));
            TPoint right = new TPoint(Double.parseDouble(delimited_r[0]), Double.parseDouble(delimited_r[1]), 
                                     Double.parseDouble(delimited_r[2]), Math.toRadians(Double.parseDouble(delimited_r[3])));
            list_l.add(left);
            list_r.add(right);                         
        }
        ArrayList<ArrayList<TPoint>> pathArray = new ArrayList<ArrayList<TPoint>>();
        pathArray.add(list_l);
        pathArray.add(list_r);

        paths.put(path, pathArray);
    }

    public void updateMaxVelocity() {
        double v = Robot.pdp.getVoltage();
        double a = 0;
        double b = 0;
        double c = 0;
        maxVelocity = a*v*v + b*v + c;
        //scrap this idea
    }
}