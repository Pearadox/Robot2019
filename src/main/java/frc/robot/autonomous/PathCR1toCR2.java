/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.*;

public class PathCR1toCR2 extends CommandGroup {

  public PathCR1toCR2(boolean mirror, double cutoffPercentage) {
    addSequential(new Follow("CR1toCR2", false, mirror, cutoffPercentage));
  }
}
