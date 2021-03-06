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
public class AutoVisionDrive extends Command {
  
  double lastError = 0;
  double error_sum = 0;
  double kp = VisionHoldOnTarget.kp;
  double ki = VisionHoldOnTarget.ki;
  double kd = VisionHoldOnTarget.kd;

  double offset = VisionHoldOnTarget.offset;  // degrees, positive is to right, negative to left
  double finishTime, speed, finishSpeed;

  boolean sawTarget;
  boolean lostTarget;
  boolean timeoutSet;
  boolean useY;

  public AutoVisionDrive(double finishTime, double speed, double finishSpeed, boolean useY) {
    requires(Robot.drivetrain);
    if (!Preferences.getInstance().containsKey("VisionHold kp")){
      Preferences.getInstance().putDouble("VisionHold kp", kp);
    }
    if (!Preferences.getInstance().containsKey("VisionHold kd")){
      Preferences.getInstance().putDouble("VisionHold kd", kd);
    }

    this.finishTime = finishTime;
    this.speed = speed;
    this.finishSpeed = finishSpeed;
    this.timeoutSet = false;
    this.useY = useY;
  }

  public AutoVisionDrive(double finishTime, double speed, double finishSpeed) {
    this(finishTime,speed, finishSpeed, false);
  }

  @Override
  protected void initialize() {
    Robot.limelight.lightOn();
    error_sum = 0;
    kp = Robot.prefs.getDouble("VisionHold kp", kp);
    kd = Robot.prefs.getDouble("VisionHold kd", kd);

    sawTarget = false;
    lostTarget = false;
    timeoutSet = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.limelight.lightOn();
    if(Robot.limelight.targetExists())  sawTarget = true;
    else if(sawTarget && !Robot.limelight.targetExists()) lostTarget = true;

    if(lostTarget) {

      if(!timeoutSet) {
        timeoutSet = true;
        setTimeout(finishTime);
      }
      Robot.drivetrain.drive(finishSpeed, finishSpeed);

    } else if(sawTarget) {
      
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

      double joystickOutput = speed;

      Robot.drivetrain.drive(output+joystickOutput, -output+joystickOutput);
    }
  }
  
  @Override
  protected boolean isFinished() {
    if (useY && Robot.limelight.getY() > 0) return true;
    if(timeoutSet && isTimedOut()) return true;
    else return false;
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
