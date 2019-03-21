/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class AutoDelay extends Command {

  public AutoDelay() {
  }

  @Override
  protected void initialize() {
    double time = Robot.prefs.getDouble("AutonomousDelay", 0);
    setTimeout(time);
  }

  @Override
  protected void execute() {
  }

  @Override
  protected boolean isFinished() {
    return isTimedOut();
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
