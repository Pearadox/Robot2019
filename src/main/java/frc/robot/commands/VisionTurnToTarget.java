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

import com.sun.java.util.jar.pack;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.JaciPathfinder;
import frc.robot.Robot;
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
  double startingWayPointx;
  double startingWayPointy;

  boolean reachedTarget;

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
    setTimeout(1.5);
    error_sum = 0;
    kp = Robot.prefs.getDouble("Vision kp", kp);
    ki = Robot.prefs.getDouble("Vision ki", ki);
    kd = Robot.prefs.getDouble("Vision kd", kd);

    reachedTarget = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      if(!Robot.limelight.targetExists()) return;

      double changeInError = lastError - Robot.limelight.getX();
      error_sum += Robot.limelight.getX();

      double P = kp * Robot.limelight.getX();
      double I = ki * error_sum;
      double D = kd * changeInError;
      lastError = Robot.limelight.getX();
      double output = P + I - D;

      // if(output > 0) output += 0.1;
      // else output -= 0.1;

      // Robot.drivetrain.setSpeed(output, -output);

      SmartDashboard.putNumber("I", ki * error_sum);
      
      // getAngle
      double getAngle = Robot.limelight.getAngle();
      // getStartingWayPointx
      double startingWayPointx = Robot.limelight.getDistance() * Math.cos(Robot.limelight.getAngle());
      // getStartingWayPointy
      double startingWayPointy = Robot.limelight.getDistance() * Math.sin(Robot.limelight.getAngle()); 
      // createShortPath
      Robot.pathfinder.createShortPath(startingWayPointx, startingWayPointy, Robot.limelight.getAngle());
      // check temp.txt file
      try {
        trajectoryReadPath("temp.txt");
      }
      catch(Exception e) {
        e.printStackTrace();
      }
      
      // driveToTarget

  }
  
  
  public void trajectoryReadPath(String path) throws IOException{
    File visionFile = new File(Robot.folder + path + "temp.txt");

    if( !visionFile.exists()) return;

    Scanner vision_scanner = new Scanner(rightFile);
  }
  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(!Robot.limelight.targetExists()) return true;

    if(Math.abs(Robot.limelight.getX()) < 1 && !reachedTarget) {
      setTimeout(0.5);
      reachedTarget = true;
      return false;
    
    }
    else if(reachedTarget && isTimedOut()) {
      double distance = Robot.limelight.getDistance()/12.;
      Scheduler.getInstance().add(new DriveForwardCommand(-distance));
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
