




/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DriveTimed_Hatch extends Command {

  double leftMotor, rightMotor, timeout;

  public DriveTimed_Hatch(double left, double right, double timeout) {
    this.leftMotor = left;
    this.rightMotor = right;
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
    if (Robot.limelight.getY() == 0 || isTimedOut()) return true;
    else return false;
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
