
package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import java.util.*;
import frc.robot.TPoint;

public class Follow extends Command {

  double ka = 0.04;
  double ka_reverse = 0.05;
  double kp = 0.03;
  double kp_reverse = 0.055;  //FIX THIS
  double kd = 0.0;
  double kh = 0.1;
  double kh_reverse = 0.02;  //FIX THIS

  boolean reverse;
  String pathName;
  ArrayList<TPoint> pathL, pathR;
  double startTime;
  double lastTime = 0;
  double lastError_r = 0;
  double lastError_l = 0;
  double startHeading =0;

  public Follow(String pathName, boolean reverse) {
    requires(Robot.drivetrain);
    this.pathName = pathName;
    this.reverse = reverse;

    // check if follow ka, follow kp, follow kd exist and put them in if they don't
    if (!Preferences.getInstance().containsKey("MP ka")){
    Preferences.getInstance().putDouble("MP ka", ka);
    }
    if (!Preferences.getInstance().containsKey("MP kp")){
      Preferences.getInstance().putDouble("MP kp", kp);
    }
    if (!Preferences.getInstance().containsKey("MP kd")){
      Preferences.getInstance().putDouble("MP kd", kd);
    }
    if (!Preferences.getInstance().containsKey("MP kh")){
      Preferences.getInstance().putDouble("MP kh", kh);
    }
  }

  @Override
  protected void initialize() {
    Robot.follower.updateMaxVelocity();
    Robot.drivetrain.zeroEncoders();
    // Get paths and put them in the TPoint lists
    ArrayList<ArrayList<TPoint>> pathPair = Robot.follower.paths.get(pathName);
    pathL = pathPair.get(0);
    pathR = pathPair.get(1);
    startTime = Timer.getFPGATimestamp();
    
    startHeading = Math.toRadians(Robot.gyro.getYaw());

    Robot.prefs = Preferences.getInstance();

    kp = Robot.prefs.getDouble("MP kp", kp);
    kd = Robot.prefs.getDouble("MP kd", kd);
    kh = Robot.prefs.getDouble("MP kh", kh);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Get all trajectory points
    double currentTime = Timer.getFPGATimestamp();
    double runTime = currentTime - startTime;
    int index = (int)Math.round(runTime / Robot.follower.dt);
    if(index >= pathL.size()) return;
    TPoint targetL = pathL.get(index);
    TPoint targetR = pathR.get(index);
    TPoint currentL = Robot.drivetrain.currentLeftTrajectoryPoint;
    TPoint currentR = Robot.drivetrain.currentRightTrajectoryPoint;

    double targetHeading_rad = targetL.heading_rad;
    if(reverse) targetHeading_rad = (targetL.heading_rad + 3 * Math.PI) % (2*Math.PI);
    if(targetHeading_rad > Math.PI) targetHeading_rad-=2*Math.PI;

    // Calculate the differences
    double start_head_target = pathL.get(0).position_ft;

    if(reverse) start_head_target = (start_head_target + 3 * Math.PI) % (2*Math.PI);
    if(start_head_target > Math.PI) start_head_target-=2*Math.PI;

    double pos_error_l = targetL.position_ft - currentL.position_ft;
    double pos_error_r = targetR.position_ft - currentR.position_ft;
    double head_error = ((targetHeading_rad - start_head_target) - (currentL.heading_rad - startHeading) % (2*Math.PI));

    double ka = reverse ? this.ka_reverse : this.ka;
    double kp = reverse ? this.kp_reverse : this.kp;
    double kh = reverse ? this.kh_reverse : this.kh;

    double leftOutput = Robot.follower.kv * targetL.velocity_ft +
                        ka * targetL.acceleration_ft +
                        kp * pos_error_l +
                        kd * ((pos_error_l - lastError_l) / 
                          (currentTime - lastTime) - targetL.velocity_ft) -
                        kh * head_error;
    double rightOutput = Robot.follower.kv*targetR.velocity_ft +
                        ka * targetR.acceleration_ft +
                        kp * pos_error_r +
                        kd * ((pos_error_r - lastError_r) / 
                          (currentTime - lastTime) - targetR.velocity_ft) +
                        kh * head_error;
    Robot.drivetrain.setSpeed(rightOutput, leftOutput);

    SmartDashboard.putNumber("V", Robot.follower.kv * targetL.velocity_ft);
    SmartDashboard.putNumber("A"  , ka*targetL.acceleration_ft);
    SmartDashboard.putNumber("P",  kp * pos_error_l);
    SmartDashboard.putNumber("D", kd * ((pos_error_l - lastError_l) / 
    (currentTime - lastTime) - targetL.velocity_ft));
    SmartDashboard.putNumber("H", kh * targetHeading_rad);

    lastTime = currentTime;
    lastError_r = pos_error_r;
    lastError_l = pos_error_l;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    double runTime = Timer.getFPGATimestamp() - startTime;
    double totalRunTime = Robot.follower.paths.get(pathName).get(0).size() * Robot.follower.dt;
    return runTime >= totalRunTime;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drivetrain.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
