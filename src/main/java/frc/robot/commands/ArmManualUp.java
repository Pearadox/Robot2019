/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.Robot;

public class ArmManualUp extends Command {
  public ArmManualUp() {
    requires(Robot.arm);
  }

  @Override
  protected void initialize() {
    // Scheduler.getInstance().add(new ArmSetAngle(48.5));
  }

  @Override
  protected void execute() {
    Robot.arm.set(0.1 + Robot.arm.calculateHoldOutput(Robot.arm.getAngle()));
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    Robot.arm.set(0);
  }

  @Override
  protected void interrupted() {
    end();
  }
}
