/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.*;

public class PathLSRtoCR1 extends CommandGroup {

  public PathLSRtoCR1(boolean mirror, double cutoffPercentage) {
    addSequential(new Follow("LSRtoCR1", false, mirror, cutoffPercentage));
  }
}
