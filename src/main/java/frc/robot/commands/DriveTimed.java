/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DriveTimed extends Command {

  double leftMotor, rightMotor, timeout;

  public DriveTimed(double timeout) {
    this.timeout = timeout;
    requires(Robot.drivetrain);
  }

  @Override
  protected void initialize() {
    setTimeout(timeout);
  }

  @Override
  protected void execute() {
    Robot.drivetrain.drive(leftMotor, rightMotor);
  }

  @Override
  protected boolean isFinished() {
    return isTimedOut();
  }

  @Override
  protected void end() {
    Robot.drivetrain.stop();
  }

  @Override
  protected void interrupted() {
    end();
  }
}
