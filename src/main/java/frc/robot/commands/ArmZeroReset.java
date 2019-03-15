/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ArmZeroReset extends Command {
  public ArmZeroReset() {
    requires(Robot.arm);
  }

  @Override
  protected void initialize() {
    
  }

  @Override
  protected void execute() {
    Robot.arm.setRawSpeed(-.07);
  }

  @Override
  protected boolean isFinished() {
    if (Robot.arm.getLimit()) 
    {
      Robot.arm.zero();
      return true;
    }
    else return false;
  }

  @Override
  protected void end() {
    Robot.arm.set(0);
    Robot.arm.zero();
  }

  @Override
  protected void interrupted() {
    end();
  }
}
