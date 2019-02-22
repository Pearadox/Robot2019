/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class BoxIntake extends Command {

  boolean timeoutSet;
  double timeout;

  public BoxIntake(double timeout) {
    requires(Robot.box);
    if(timeout != -1) {
      this.timeoutSet = true;
      this.timeout = timeout;
    } 
  }

  public BoxIntake() {
    this(-1);
  }

  @Override
  protected void initialize() {
    if(timeoutSet) setTimeout(timeout);
  }

  @Override
  protected void execute() {
    Robot.box.set(0.3);
  }

  @Override
  protected boolean isFinished() {
    if(timeoutSet) return isTimedOut();
    else return false;
  }

  @Override
  protected void end() {
    Robot.box.set(0);
  }

  @Override
  protected void interrupted() {
    end();
  }
}
