/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.pathfollowing.*;
import frc.robot.utilities.Vector;
import frc.robot.Robot;

public class PPFollow extends Command {

  final double lookaheadDistance = 1.5;  // feet, smaller is better for curvy
  double ka = 0.04;
  double kp = 0.15;

  double currentX = 0, currentY = 0, startingHeading = 0, lastLeft = 0, lastRight = 0;
  double lastVelocity_l = 0, lastVelocity_r, lastTime = 0;
  int lastClosestPointIndex = 0;
  int lookaheadIndex = 0;
  Vector lookaheadPoint = new Vector(0, 0);

  ArrayList<PPPoint> trajectory = new ArrayList<>();

  public PPFollow(double x, double y, double headingCorrection) {
    requires(Robot.drivetrain);
    trajectory = Robot.pp.generatePath(x, y, headingCorrection);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    startingHeading = Robot.gyro.getYaw();
    lastLeft = Robot.drivetrain.getLeftEncoderFeet();
    lastRight = Robot.drivetrain.getRightEncoderFeet();
    lastTime = Timer.getFPGATimestamp();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // estimate current odometry
    double relativeHeading = Robot.gyro.getYaw() - startingHeading;
    double deltaLeft = Robot.drivetrain.getLeftEncoderFeet() - lastLeft;
    double deltaRight = Robot.drivetrain.getRightEncoderFeet() - lastRight;
    double distance = (deltaLeft + deltaRight) / 2;
    currentX += distance * Math.cos(Math.toRadians(relativeHeading));
    currentY += distance * Math.sin(Math.toRadians(relativeHeading));

    // calculate closest point
    double smallestDistance = Double.MAX_VALUE;
    int closestPointIndex = lastClosestPointIndex;
    for(int i = lastClosestPointIndex; i < trajectory.size(); i++) {
      if(smallestDistance > trajectory.get(i).calculateDistance(currentX, currentY)) {
        smallestDistance = trajectory.get(i).calculateDistance(currentX, currentY);
        closestPointIndex = i;
      }
    }
    PPPoint closestPoint = trajectory.get(closestPointIndex);
    // find furthest lookahead point
    for(int i = lookaheadIndex; i < trajectory.size()-1; i++) {

      Vector C = new Vector(currentX, currentY);  // center of circle, robot location
      PPPoint segStart = trajectory.get(i);
      PPPoint segEnd = trajectory.get(i+1);
      Vector E = new Vector(segStart.x, segStart.y);
      Vector L = new Vector(segEnd.x, segEnd.y);
      double r = lookaheadDistance;
      
      Vector d = L.subtract(E);  // direction vector of ray, from start to end
      Vector f = E.subtract(C);  // vector from center sphere to ray start

      double a = d.dp(d);
      double b = 2 * f.dp(d);
      double c = f.dp(f) - r*r;
      double discriminant = b*b - 4*a*c;

      if(discriminant < 0) continue;  // no intersection
      else {
        discriminant = Math.sqrt(discriminant);
        double t1 = (-b - discriminant) / (2*a);
        double t2 = (-b + discriminant) / (2*a);
        
        if(t1 >= 0 && t2 <= 1) {
          // return t1 intersection
          lookaheadPoint = E.add(d.multiply(t1));
        }
        if(t2 >= 0 && t2 <= 1) {
          // return t2 intersection
          lookaheadPoint = E.add(d.multiply(t2));
        }
        // otherwise, no intersection
        continue;
      }
    }

    // calculate curvature

    // get coefficients of ax+by+c=0, formula of equation of robot line
    double a = -Math.tan(Math.toRadians(relativeHeading));
    double b = 1;
    double c = Math.tan(Math.toRadians(relativeHeading)) * currentX - currentY;

    double x = Math.abs(a*lookaheadPoint.i + b*lookaheadPoint.j + c) / Math.sqrt(a*a + b*b);

    // use x to calculate curvature of path
    double curvature = 2 * x / (lookaheadDistance * lookaheadDistance);

    //determine if lookahead point is on the left or right side of the robot
    Vector R = new Vector(currentX, currentY);  // robot location
    Vector L = lookaheadPoint;  // lookahead point
    double side = Math.signum(Math.sin(relativeHeading) * (L.i-R.i) - Math.cos(relativeHeading) * (L.j-R.j));

    // add sign to curvature
    curvature *= side;
    
    // calculate wheel velocities
    double V = closestPoint.velocity;  // target velocity
    double C = curvature;  // curvature of arc
    double T = Robot.follower.WHEEL_BASE_WIDTH;  // track width

    double left = V * (2 + C*T) / 2.;
    double right = V * (2 - C*T) / 2.;

    // calculate acceleration, take derivative of velocity
    double dt = Timer.getFPGATimestamp() - lastTime;
    double dV_l = left - lastVelocity_l;
    double dV_r = right - lastVelocity_r;
    lastVelocity_l = left;
    lastVelocity_r = right;
    lastTime = Timer.getFPGATimestamp();
    double A_l = dV_l / dt;
    double A_r = dV_r / dt;

    calculate feedforward and feedback terms
    double FF_l = Robot.follower.kv * left + ka * A_l;
    double FF_r = Roobt.follower.kv * right + ka * A_r;
    double measured_l = Robot.drivetrain.currentLeftTrajectoryPoint.velocity_ft;
    double measured_r = Robot.drivetrain.currentRightTrajectoryPoint.velocity_ft;
    double FB_l = kp * (left - measured_l);
    double FB_r = kp * (right - measured_r);

    double leftOutput = FF_l + FB_l;
    double rightOutput = FF_r + FB_r;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
