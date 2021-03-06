/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class IntakeIntake extends Command {

  boolean hasTimeout;
  double timeout;

  public IntakeIntake(double timeout) {
    requires(Robot.intake);
    if(timeout != -1) {
      hasTimeout = true;
      this.timeout = timeout;
    }
  }

  public IntakeIntake() {
    this(-1);
  }

  @Override
  protected void initialize() {
    if(hasTimeout) setTimeout(timeout);
  }
  
  @Override
  protected void execute() {
    Robot.intake.set(.5);
  }

  @Override
  protected boolean isFinished() {
    if(hasTimeout) return isTimedOut();
    else return false;
  }

  @Override
  protected void end() {
    Robot.intake.set(0);
  }

  @Override
  protected void interrupted() {
    end();
  }
}
