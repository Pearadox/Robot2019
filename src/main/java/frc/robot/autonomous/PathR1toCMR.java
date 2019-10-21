/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.*;

public class PathR1toCMR extends CommandGroup {
  /**
   * Add your docs here.
   */
  public PathR1toCMR(boolean mirror, double cutoffPercentage) {
    addSequential(new Follow("R1toCMR", false, mirror));
    //addSequential(new Follow("LSRtoCML2of2", true, mirror, cutoffPercentage));
  }
}
