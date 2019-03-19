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


public class VisionHoldOnTarget extends Command {
  
  double lastError = 0;
  double error_sum = 0;
  public static double kp = 0.08;
  public static double ki = 0.0;
  public static double kd = 0.1;

  public static double offset = 0;  // degrees, positive is to right, negative to left

  boolean reachedTarget;

  public VisionHoldOnTarget() {
    requires(Robot.drivetrain);
    if (!Preferences.getInstance().containsKey("VisionHold kp")){
      Preferences.getInstance().putDouble("VisionHold kp", kp);
    }
    if (!Preferences.getInstance().containsKey("VisionHold ki")){
      Preferences.getInstance().putDouble("VisionHold ki", ki);
    }
    if (!Preferences.getInstance().containsKey("VisionHold kd")){
      Preferences.getInstance().putDouble("VisionHold kd", kd);
    }
  }

  @Override
  protected void initialize() {
    Robot.limelight.lightOn();
    error_sum = 0;
    kp = Robot.prefs.getDouble("VisionHold kp", kp);
    ki = Robot.prefs.getDouble("VisionHold ki", ki);
    kd = Robot.prefs.getDouble("VisionHold kd", kd);

    reachedTarget = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

      double getX = Robot.limelight.getX() - offset;

      double changeInError = lastError - getX;
      error_sum += getX;

      double P = kp * getX;
      double I = ki * error_sum;
      double D = kd * changeInError;
      lastError = getX;
      double output = P + I - D;

      if(output > 0) output += 0.0;
      else output -= 0.0;

      if(!Robot.limelight.targetExists()) output = 0;

      double joystickOutput = -Robot.oi.joystick.getRawAxis(1);

      boolean reduce = Robot.oi.joystick.getRawButton(1);
      if(reduce) joystickOutput *= 0.5;

      Robot.drivetrain.drive(output+joystickOutput, -output+joystickOutput);
  }
  
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drivetrain.stop();
    // Robot.limelight.lightOff();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
