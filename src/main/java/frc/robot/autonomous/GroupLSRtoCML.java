/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.*;

public class GroupLSRtoCML extends CommandGroup {
  /**
   * Add your docs here.
   */
  public GroupLSRtoCML(boolean mirror) {
    addSequential(new PathLSRtoCML(mirror, .7));
    addSequential(new AutoVisionDrive(1.5, -0.45, -.2));
  }
}
