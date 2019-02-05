package frc.robot.pathfollowing;

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

    double MAX_VELOCITY = 8;
    double acceleration = 3;
    double max_jerk = 50;
    

    public ArrayList<ArrayList<TPoint>> createShortPath(double x, double y, double headingCorrection) {
        
        Waypoint[] points = new Waypoint[] {
            new Waypoint(-x, -y, -headingCorrection),
            new Waypoint(0, 0, 0)
        };
        
        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_LOW, 
                                                            Robot.follower.dt, MAX_VELOCITY, acceleration, max_jerk);                                    
        Trajectory trajectory = Pathfinder.generate(points, config);
        TankModifier modifier = new TankModifier(trajectory).modify(Robot.follower.WHEEL_BASE_WIDTH);
        Segment[] leftSegments = modifier.getLeftTrajectory().segments;
        Segment[] rightSegments = modifier.getRightTrajectory().segments;

        ArrayList<ArrayList<TPoint>> lists = new ArrayList<>();
        ArrayList<TPoint> left = new ArrayList<>();
        ArrayList<TPoint> right = new ArrayList<>();

          
        for(int i = 0; i < leftSegments.length; i++) {
            Segment lSeg = leftSegments[i];
            Segment rSeg = rightSegments[i];
            // TPoint l = new TPoint(lSeg.position, lSeg.velocity, lSeg.acceleration, lSeg.heading);
            // TPoint r = new TPoint(rSeg.position, rSeg.velocity, rSeg.acceleration, rSeg.heading);

            TPoint l = new TPoint(-rSeg.position, -rSeg.velocity, -rSeg.acceleration, lSeg.heading);
            TPoint r = new TPoint(-lSeg.position, -lSeg.velocity, -lSeg.acceleration, rSeg.heading);

            left.add(l);
            right.add(r);
        }

        lists.add(left);
        lists.add(right);
        
        return lists;
    }

    public double[][] createPositionalPath(double x, double y, double headingCorrection, double dt)
    {
        Waypoint[] points = new Waypoint[] {
            new Waypoint(-x, -y, -headingCorrection),
            new Waypoint(0, 0, 0)
        };
        
        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_FAST, 
                                                            dt, MAX_VELOCITY, acceleration, max_jerk);                                    
        Trajectory trajectory = Pathfinder.generate(points, config);
        Segment[] segments = trajectory.segments;
        double[][] path = new double[segments.length][2];
        double totalX = 0;
        double totalY = 0;
        double lastPosition = 0;
        for(int i = 0; i < segments.length; i++) {
            Segment seg = segments[i];
            path[i][0] = totalX;
            path[i][1] = totalY;
            totalX += (seg.position-lastPosition) * Math.cos(seg.heading);
            totalY += (seg.position-lastPosition) * Math.sin(seg.heading);
            lastPosition = seg.position;
        }
        return path;
    }
}