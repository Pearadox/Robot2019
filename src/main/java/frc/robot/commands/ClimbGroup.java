/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ClimbGroup extends CommandGroup {

  static double climbingUpAngle = 50.8;
  static double fallingDownAngle = 40;

  public ClimbGroup() {
    addSequential(new ArmSetAngle(true));
  }
}
