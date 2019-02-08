/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class DriveWithJoystick extends Command {

  public DriveWithJoystick() {
    requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    double forwardJoystick = Robot.oi.joystick.getRawAxis(1);
    double rotateJoystick = Robot.oi.joystick.getRawAxis(2);
    boolean reverse = Robot.reverseDrivetrain;
    boolean reduce = Robot.oi.joystick.getRawButton(1);

    if(Math.abs(forwardJoystick)<.15) {
			forwardJoystick = 0;
		}
    if(reverse) {
      forwardJoystick *= -1;
    }
    if(reduce) {
      forwardJoystick *= 0.5;
      rotateJoystick *= 0.3;
    }

    // Robot.drivetrain.driveWithJoystick(forwardJoystick, rotateJoystick);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
