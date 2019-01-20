/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class VisionTurnToTarget extends Command {
  
  double lastError = 0;
  boolean stopLoop = false;

  public VisionTurnToTarget() {
    requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    stopLoop = false;
    // if(!Robot.limelight.targetExists()) stopLoop = true;
    setTimeout(1.5);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      if(stopLoop) return;
      double kp = 0.021;
      double kd = 0.15;
      double P = kp * Robot.limelight.getX();
      double changeInError = lastError - Robot.limelight.getX();
      double D = kd * changeInError;
      lastError = Robot.limelight.getX();
      double output = P - D + .1;
      Robot.drivetrain.setSpeed(output, -output);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(Math.abs(Robot.limelight.getX()) < 1) {
      double distance = Robot.limelight.getDistance()/12.;
      Scheduler.getInstance().add(new DriveForwardCommand(-distance));
      return true;
    }
    return isTimedOut() | stopLoop;
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
