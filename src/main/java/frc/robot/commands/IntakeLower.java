/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class IntakeLower extends Command {

  double timeoutIfNecessary;

  public IntakeLower() {
    this(-1);
  }

  public IntakeLower(double timeoutIfNecessary) {
    this.timeoutIfNecessary = timeoutIfNecessary;
  }

  @Override
  protected void initialize() {
    if(timeoutIfNecessary > 0) {
      if(!Robot.intake.isLow()) {
        setTimeout(timeoutIfNecessary);
      }
      else setTimeout(0);
    }
    Robot.intake.lower();
  }

  @Override
  protected void execute() {
  }

  @Override
  protected boolean isFinished() {
    if(timeoutIfNecessary > 0)
      return isTimedOut();
    else return true;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
