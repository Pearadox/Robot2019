/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.DriveTimed;
import frc.robot.commands.DriveForward;
import frc.robot.commands.Follow;
import frc.robot.commands.TurnAbsolute;

public class PathCMLtoLSL extends CommandGroup {
  /**
   * Add your docs here.
   */
  public PathCMLtoLSL(boolean mirror, double cutoffPercentage) {
    addSequential(new Follow("CMLtoLSL1of2", false, mirror));
    addSequential(new Follow("CMLtoLSL2of2", true, mirror, cutoffPercentage));
  }
}
