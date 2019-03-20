/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.*;

public class PathLSRtoCR extends CommandGroup {

  public PathLSRtoCR(int cargoTarget, boolean mirror, double cutoffPercentage) {
    addSequential(new Follow("LSRtoCR" + cargoTarget + "1of2", false, mirror));
    addSequential(new Follow("LSRtoCR" + cargoTarget + "2of2", false, mirror, cutoffPercentage));
  }
}
