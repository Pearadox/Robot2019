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
        SmartDashboard.putNumber("part", 1);
        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 
                                                            Robot.follower.dt, MAX_VELOCITY, acceleration, max_jerk);
        Trajectory trajectory = Pathfinder.generate(points, config);
        TankModifier modifier = new TankModifier(trajectory).modify(WHEEL_BASE_WIDTH);
        SmartDashboard.putNumber("part", 2);
        Segment[] leftSegments = modifier.getLeftTrajectory().segments;
        Segment[] rightSegments = modifier.getRightTrajectory().segments;

        ArrayList<ArrayList<TPoint>> lists = new ArrayList<>();
        ArrayList<TPoint> left = new ArrayList<>();
        ArrayList<TPoint> right = new ArrayList<>();

        for(int i = 0; i < leftSegments.length; i++) {
            SmartDashboard.putNumber("part", i);
            Segment lSeg = leftSegments[i];
            Segment rSeg = rightSegments[i];
            TPoint l = new TPoint(lSeg.position, lSeg.velocity, lSeg.acceleration, lSeg.heading);
            TPoint r = new TPoint(rSeg.position, rSeg.velocity, rSeg.acceleration, rSeg.heading);

            left.add(l);
            right.add(r);
        }

        SmartDashboard.putNumber("Size", left.size());

        lists.add(right);
        lists.add(left);
        
        return lists;
    }
}