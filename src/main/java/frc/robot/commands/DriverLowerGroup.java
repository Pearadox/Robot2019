/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class DriverLowerGroup extends CommandGroup {
  
  public DriverLowerGroup() {
    addSequential(new IntakeLower());
    addSequential(new Delay(1.5));
    addSequential(new ArmGoLow());
    addSequential(new IntakeBoth(false));
  }
}
