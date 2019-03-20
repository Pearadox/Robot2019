/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.Follow;

public class AutonomousRtoCMRtoCR extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutonomousRtoCMRtoCR(int startingLevel, int cargoTarget, boolean mirror) {
    addSequential(new GroupRtoCMRtoLSR(startingLevel, mirror));
    addSequential(new GroupLSRtoCR(cargoTarget, mirror));
  }
}
