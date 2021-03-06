/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.*;

public class GroupLSRtoCR1 extends CommandGroup {
  /**
   * Add your docs here.
   */
  public GroupLSRtoCR1(boolean mirror) {
    addSequential(new PathLSRtoCR1(mirror, 0));
    addSequential(new AutoVisionDrive(0, -0.4, -.2));
    addSequential(new DriveTimed(-.2,-.2,.18));
    addSequential(new MothOpen());
  }
}
