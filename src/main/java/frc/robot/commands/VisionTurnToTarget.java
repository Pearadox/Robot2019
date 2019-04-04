/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.IOException;
import java.io.*;
import java.util.*;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.pathfollowing.*;
import frc.robot.*;
import frc.robot.subsystems.Limelight;
import jaci.pathfinder.Pathfinder;

/**
 * An example command.  You can replace me with your own command.
 */
public class VisionTurnToTarget extends Command {
  
  double lastError = 0;
  double error_sum = 0;
  double kp = 0.021;
  double ki = 0.0;
  double kd = 0.15;

  boolean reachedTarget;
  boolean inTeleop;

  public VisionTurnToTarget() {
    requires(Robot.drivetrain);
    if (!Preferences.getInstance().containsKey("Vision kp")){
      Preferences.getInstance().putDouble("Vision kp", kp);
    }
    if (!Preferences.getInstance().containsKey("Vision ki")){
      Preferences.getInstance().putDouble("Vision ki", ki);
    }
    if (!Preferences.getInstance().containsKey("Vision kd")){
      Preferences.getInstance().putDouble("Vision kd", kd);
    }
  }

  @Override
  protected void initialize() {
    setTimeout(.5);
    error_sum = 0;
    kp = Robot.prefs.getDouble("Vision kp", kp);
    ki = Robot.prefs.getDouble("Vision ki", ki);
    kd = Robot.prefs.getDouble("Vision kd", kd);

    reachedTarget = false;
    inTeleop = true;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if(Robot.limelight.targetExists()) inTeleop = false;
    if(inTeleop) {
      double joystickForward = Math.min(Robot.oi.joystick.getRawAxis(1), .4);
      double joystickRotate = Math.min(Robot.oi.joystick.getRawAxis(2), .6);
      Robot.drivetrain.arcadeDrive(joystickForward, joystickRotate);
    }
    else {
      double changeInError = lastError - Robot.limelight.getX();
      error_sum += Robot.limelight.getX();

      double P = kp * Robot.limelight.getX();
      double I = ki * error_sum;
      double D = kd * changeInError;
      lastError = Robot.limelight.getX();
      double output = P + I - D;

      if(output > 0) output += 0.1;
      else output -= 0.1;

      Robot.drivetrain.drive(output, -output);
    }
  }
  
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(inTeleop) return false;
    if(isTimedOut()) return true;
    if(!Robot.limelight.targetExists()) return true;

    if(Math.abs(Robot.limelight.getX()) < 1 && !reachedTarget) {
      setTimeout(0.5);
      reachedTarget = true;
      return false;
    }
    else if(reachedTarget && isTimedOut()) {
      Robot.drivetrain.stop();
      return true;
    }
    return isTimedOut();
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
    Robot.drivetrain.stop();
  }
}
