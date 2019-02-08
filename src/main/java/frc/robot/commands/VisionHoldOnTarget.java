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
public class VisionHoldOnTarget extends Command {
  
  double lastError = 0;
  double error_sum = 0;
  double kp = 0.01;
  double ki = 0.0;
  double kd = 0.1;

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
    setTimeout(1.5);
    error_sum = 0;
    kp = Robot.prefs.getDouble("VisionHold kp", kp);
    ki = Robot.prefs.getDouble("VisionHold ki", ki);
    kd = Robot.prefs.getDouble("VisionHold kd", kd);

    reachedTarget = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      if(!Robot.limelight.targetExists()) return;

      double getX = Robot.limelight.getX();

      double changeInError = lastError - Robot.limelight.getX();
      error_sum += Robot.limelight.getX();

      double P = kp * Robot.limelight.getX();
      double I = ki * error_sum;
      double D = kd * changeInError;
      lastError = Robot.limelight.getX();
      double output = P + I - D;

      if(output > 0) output += 0.0;
      else output -= 0.0;

      double joystickOutput = -Robot.oi.joystick.getRawAxis(1);

      Robot.drivetrain.setSpeed(output+joystickOutput, -output+joystickOutput);
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
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
