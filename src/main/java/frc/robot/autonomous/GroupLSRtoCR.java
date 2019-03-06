/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.*;

public class GroupLSRtoCR extends CommandGroup {
  /**
   * Add your docs here.
   */
  public GroupLSRtoCR(int cargoTarget, boolean mirror) {
    addSequential(new PathLSRtoCR(cargoTarget, mirror));
  }
}
