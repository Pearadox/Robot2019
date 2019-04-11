/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class TurnAbsolute extends Command {

  double absoluteTarget;

  public TurnAbsolute(double angle) {
    this.absoluteTarget = angle;
  }

  @Override
  protected void initialize() {
    double toTurn = (this.absoluteTarget - Robot.gyro.getYaw()) % (360);

    if(toTurn > 180) toTurn -= 360;
    else if(toTurn < -180) toTurn += 360;
    
    if(toTurn < 0) Scheduler.getInstance().add(new TurnRelative(Math.abs(toTurn)));
    else Scheduler.getInstance().add(new TurnRelative(-Math.abs(toTurn)));
  }

  @Override
  protected void execute() {
  }

  @Override
  protected boolean isFinished() {
    return !Robot.isFollowingPath;
  }

  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
