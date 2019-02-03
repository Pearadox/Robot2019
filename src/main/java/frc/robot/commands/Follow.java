
package frc.robot.commands;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

import java.util.*;
import frc.robot.TPoint;

public class Follow extends Command {

  double ka = 0.04;
  double kp = 0.15;
  double kp_reverse = 0.07;
  double kd = 0.0;
  double kh = -.08;
  double kh_reverse = 0.017;  //FIX THIS

  boolean reverse, mirror;
  boolean ignoreHeading = false;
  String pathName;
  ArrayList<TPoint> pathL, pathR;
  double startTime;
  double lastTime = 0;
  double lastError_r = 0;
  double lastError_l = 0;
  double startHeading = 0;

  public Follow(boolean ignoreHeading, boolean reverse, boolean mirror) {
    requires(Robot.drivetrain);

    this.ignoreHeading = ignoreHeading;
    this.reverse = reverse;
    this.mirror = mirror;

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
    if (!Preferences.getInstance().containsKey("MP kh_reverse")){
      Preferences.getInstance().putDouble("MP kh_reverse", kh_reverse);
    }
    if (!Preferences.getInstance().containsKey("MP kp_reverse")){
      Preferences.getInstance().putDouble("MP kp_reverse", kp_reverse);
    }
  }

  public Follow(ArrayList<ArrayList<TPoint>> list, boolean ignoreHeading) {
    this(ignoreHeading, false, false);
    pathL = list.get(0);
    pathR = list.get(1);
  }

  public Follow(String pathName, boolean reverse, boolean mirror) {
    this(false, reverse, mirror);
    this.pathName = pathName;

    ArrayList<ArrayList<TPoint>> pathPair = Robot.follower.paths.get(this.pathName);
    pathL = pathPair.get(0);
    pathR = pathPair.get(1);
  }

  @Override
  protected void initialize() {
    Robot.drivetrain.zeroEncoders();
    // Get paths and put them in the TPoint lists
    startTime = Timer.getFPGATimestamp();
    
    startHeading = Math.toRadians(Robot.gyro.getYaw());

    Robot.prefs = Preferences.getInstance();

    kp = Robot.prefs.getDouble("MP kp", kp);
    ka = Robot.prefs.getDouble("MP ka", ka);
    kh = Robot.prefs.getDouble("MP kh", kh);

    kp_reverse = Robot.prefs.getDouble("MP kp_reverse", kp_reverse);
    kh_reverse = Robot.prefs.getDouble("MP kh_reverse", kh_reverse);
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
    if(ignoreHeading) {
      double difference = targetL.position_ft-targetR.position_ft;
      double halfTurnFeet = RobotMap.halfTurn * RobotMap.feetPerTick;
      targetHeading_rad = difference/halfTurnFeet * Math.PI;
    }

    double pos_targetL = (reverse^mirror) ? targetR.position_ft : targetL.velocity_ft;
    double vel_targetL = (reverse^mirror) ? targetR.position_ft : targetL.velocity_ft;
    double accel_targetL = (reverse^mirror) ? targetR.position_ft : targetL.acceleration_ft;
    double pos_targetR = (reverse^mirror) ? targetL.position_ft : targetR.position_ft;
    double vel_targetR = (reverse^mirror) ? targetL.position_ft : targetR.velocity_ft;
    double accel_targetR = (reverse^mirror) ? targetL.position_ft : targetR.acceleration_ft;

    if(reverse) {
      pos_targetL *= -1;
      vel_targetL *= -1;
      accel_targetL *= -1;
      pos_targetR *= -1;
      vel_targetR *= -1;
      accel_targetR *= -1;
    }

    // Calculate the differences
    double start_head_target = pathL.get(0).position_ft;

    double pos_error_l = targetL.position_ft - currentL.position_ft;
    double pos_error_r = targetR.position_ft - currentR.position_ft;
    double head_error = (targetHeading_rad - start_head_target) - (currentL.heading_rad - startHeading);

    if(mirror) head_error *= -1;

    head_error %= (2*Math.PI);
    if(head_error > Math.PI) head_error-=2*Math.PI;
    if(head_error < -Math.PI) head_error+=2*Math.PI;

    double kp = reverse ? this.kp_reverse : this.kp;
    double kh = reverse ? this.kh_reverse  : this.kh;

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
    Robot.drivetrain.setSpeed(leftOutput, rightOutput);

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
    double totalRunTime = pathL.size() * Robot.follower.dt;
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
