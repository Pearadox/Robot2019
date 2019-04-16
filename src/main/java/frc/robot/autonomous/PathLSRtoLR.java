/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.DriveForward;
import frc.robot.commands.Follow;
import frc.robot.commands.*;

public class PathLSRtoLR extends CommandGroup {
  /**
   * Add your docs here.
   */
  public PathLSRtoLR(int rocketTarget, boolean mirror, double cutoffPercentage) {
    addSequential(new Follow("LSRtoLR" + rocketTarget, false, mirror, cutoffPercentage));
  }
}
