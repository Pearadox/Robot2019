
package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.pathfollowing.*;
import frc.robot.utilities.Vector;
import frc.robot.Robot;

public class PPFollow extends Command {

  final double lookaheadDistance = 1.5;  // feet, smaller is better for curvy
  double ka = 0.001;
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

    // if (!Preferences.getInstance().containsKey("PP ka")){
    //   Preferences.getInstance().putDouble("PP ka", ka);
    // }
    // if (!Preferences.getInstance().containsKey("PP kp")){
    //   Preferences.getInstance().putDouble("PP kp", kp);
    // }
  }

  @Override
  protected void initialize() {
    startingHeading = Robot.gyro.getYaw();
    lastLeft = Robot.drivetrain.getLeftEncoderFeet();
    lastRight = Robot.drivetrain.getRightEncoderFeet();
    lastTime = Timer.getFPGATimestamp();

    kp = Robot.prefs.getDouble("PP kp", kp);
    ka = Robot.prefs.getDouble("PP ka", ka);
  }

  @Override
  protected void execute() {
    // estimate current odometry
    double relativeHeading = Robot.gyro.getYaw() - startingHeading;
    double deltaLeft = Robot.drivetrain.getLeftEncoderFeet() - lastLeft;
    double deltaRight = Robot.drivetrain.getRightEncoderFeet() - lastRight;
    double distance = (deltaLeft + deltaRight) / 2;
    currentX += distance * Math.cos(Math.toRadians(relativeHeading));
    currentY += distance * Math.sin(Math.toRadians(relativeHeading));
    lastLeft = Robot.drivetrain.getLeftEncoderFeet();
    lastRight = Robot.drivetrain.getRightEncoderFeet();

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
        
        if(t1 >= 0 && t1 <= 1) {
          // return t1 intersection
          lookaheadPoint = E.add(d.multiply(t1));
          lookaheadIndex = i;
          continue;
        }
        if(t2 >= 0 && t2 <= 1) {
          // return t2 intersection
          lookaheadPoint = E.add(d.multiply(t2));
          lookaheadIndex = i;
          continue;
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

    // calculate feedforward and feedback terms
    double FF_l = Robot.follower.kv * left + ka * A_l;
    double FF_r = Robot.follower.kv * right + ka * A_r;
    double measured_l = Robot.drivetrain.currentLeftTrajectoryPoint.velocity_ft;
    double measured_r = Robot.drivetrain.currentRightTrajectoryPoint.velocity_ft;
    double FB_l = kp * (left - measured_l);
    double FB_r = kp * (right - measured_r);

    // double leftOutput = FF_l + FB_l;
    // double rightOutput = FF_r + FB_r;

    double leftOutput = FF_l;
    double rightOutput = FF_r;

    Robot.drivetrain.drive(leftOutput, rightOutput);

    SmartDashboard.putNumber("PP VelocityL", left);
    SmartDashboard.putNumber("PP VelocityR", right);
    SmartDashboard.putNumber("PP FeedForward", FF_l);
    SmartDashboard.putNumber("PP FeedBack", FB_l);
    SmartDashboard.putNumber("PP Current X", currentX);
    SmartDashboard.putNumber("PP Current Y", currentY);
    SmartDashboard.putNumber("PP Lookahead Index", lookaheadIndex);
    SmartDashboard.putString("PP point", trajectory.get(1).x + " " + trajectory.get(1).y);
  }

  @Override
  protected boolean isFinished() {
    // finishes once lookahead point is the last point on the trajectory
    return lookaheadIndex+1 == trajectory.size();
  }

  @Override
  protected void end() {
    Robot.drivetrain.stop();
  }

  @Override
  protected void interrupted() {
    end();
  }
}
