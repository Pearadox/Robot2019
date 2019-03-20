/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.*;

public class GroupLSRtoRR2 extends CommandGroup {
  /**
   * Add your docs here.
   */
  public GroupLSRtoRR2(boolean mirror) {
    addSequential(new PathLSRtoRR(2, mirror, .7));
    addSequential(new AutoVisionDrive(2, -0.55, -.2));
    addSequential(new MothClose());
  }
}
