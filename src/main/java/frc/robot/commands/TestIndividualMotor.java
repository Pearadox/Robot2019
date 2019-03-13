/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class TestIndividualMotor extends Command {

  int motor;
  boolean onRight;
  double time;

  public TestIndividualMotor(int motor, boolean onRight, double time) {
    requires(Robot.drivetrain);
    this.motor = motor;
    this.onRight = onRight;
  }

  @Override
  protected void initialize() {
    setTimeout(time);
    if(onRight) {
      if(motor == 1) Robot.drivetrain.setRightMaster(motor);
      else if(motor == 2) Robot.drivetrain.setRightSlave1(motor);
      else if(motor == 3) Robot.drivetrain.setRightSlave2(motor);
    }
    else {
      if(motor == 1) Robot.drivetrain.setLeftMaster(motor);
      else if(motor == 2) Robot.drivetrain.setLeftSlave1(motor);
      else if(motor == 3) Robot.drivetrain.setLeftSlave2(motor);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
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
    end();
  }
}
