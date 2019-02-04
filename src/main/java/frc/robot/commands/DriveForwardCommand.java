/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.*;
import frc.robot.pathfollowing.*;
import frc.robot.pathfollowing.TrajectoryGenerator;
import java.util.*;

public class DriveForwardCommand extends Command {

  double ka = 0.055;
  double kp = 0.0;
  double kd = 0.0;
  double kh = 0.0;

  ArrayList<TPoint> pathL, pathR;
  double startTime;
  double lastTime = 0;
  double lastError_r = 0;
  double lastError_l = 0;
  double startHeading =0;

  double maxVelocity = 12;
  double acceleration = 5;

  double feet;

  public DriveForwardCommand(double feet) {
    requires(Robot.drivetrain);
    this.feet = feet*5/3.7;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    pathL = TrajectoryGenerator.getTrajectory(feet, .02, maxVelocity, acceleration);
    pathR = TrajectoryGenerator.getTrajectory(feet, .02, maxVelocity, acceleration);

    startTime = Timer.getFPGATimestamp();
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

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
        // Get all trajectory points
        double currentTime = Timer.getFPGATimestamp();
        double runTime = currentTime - startTime;
        int index = (int)Math.round(runTime / .02);
        if(index >= pathL.size()) return;
        TPoint targetL = pathL.get(index);
        TPoint targetR = pathR.get(index);
        TPoint currentL = Robot.drivetrain.currentLeftTrajectoryPoint;
        TPoint currentR = Robot.drivetrain.currentRightTrajectoryPoint;
    
        // Calculate the differences
        double pos_error_l = targetL.position_ft-currentL.position_ft;
        double pos_error_r = targetR.position_ft-currentR.position_ft;
        double head_error = targetL.heading_rad-currentL.heading_rad - startHeading;
    
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
        SmartDashboard.putNumber("A", ka*targetL.acceleration_ft);
        SmartDashboard.putNumber("P",  kp * pos_error_l);
        SmartDashboard.putNumber("D", kd * ((pos_error_l - lastError_l) / 
        (currentTime - lastTime) - targetL.velocity_ft));
        SmartDashboard.putNumber("H", -kh * head_error);
    
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
