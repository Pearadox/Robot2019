package frc.robot;

import frc.robot.Robot;

import java.io.FileWriter;
import java.io.*;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.modifiers.TankModifier;

public class JaciPathfinder {

    double MAX_VELOCITY = Robot.follower.maxVelocity;
    double acceleration = 5;
    double max_jerk = 200;
    double WHEEL_BASE_WIDTH = 2.3;

    public ArrayList<ArrayList<TPoint>> createShortPath(double x, double y, double headingCorrection) {
        
        Waypoint[] points = new Waypoint[] {
            new Waypoint(-x, -y, -headingCorrection),
            new Waypoint(0, 0, 0)
        };
// /*
        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_FAST, 
                                                            Robot.follower.dt, MAX_VELOCITY, acceleration, max_jerk);
                                                            // */

        // Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 1.7, 2.0, 60.0);                                        
        Trajectory trajectory = Pathfinder.generate(points, config);
        TankModifier modifier = new TankModifier(trajectory).modify(WHEEL_BASE_WIDTH);
        Segment[] leftSegments = modifier.getLeftTrajectory().segments;
        Segment[] rightSegments = modifier.getRightTrajectory().segments;

        ArrayList<ArrayList<TPoint>> lists = new ArrayList<>();
        ArrayList<TPoint> left = new ArrayList<>();
        ArrayList<TPoint> right = new ArrayList<>();

        
        try {
            PrintWriter pw = new PrintWriter(new FileWriter("/home/lvuser/aweo.txt"));
            pw.println("path");
          
        for(int i = 0; i < leftSegments.length; i++) {
            SmartDashboard.putNumber("part", i);
            Segment lSeg = leftSegments[i];
            Segment rSeg = rightSegments[i];
            // TPoint l = new TPoint(lSeg.position, lSeg.velocity, lSeg.acceleration, lSeg.heading);
            // TPoint r = new TPoint(rSeg.position, rSeg.velocity, rSeg.acceleration, rSeg.heading);

            TPoint l = new TPoint(-rSeg.position, -rSeg.velocity, -rSeg.acceleration, lSeg.heading);
            TPoint r = new TPoint(-lSeg.position, -lSeg.velocity, -lSeg.acceleration, rSeg.heading);

            left.add(l);
            right.add(r);

            pw.println(lSeg.position + "," + rSeg.position + " " + lSeg.heading);
        }
        
            pw.close();
        } catch(Exception e) {}

        lists.add(left);
        lists.add(right);
        
        return lists;
    }
}