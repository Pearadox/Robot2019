/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.*;

public class GroupRtoRR1toLSR extends CommandGroup {
  /**
   * Add your docs here.
   */
  public GroupRtoRR1toLSR(int startingLevel, boolean mirror) {
    addSequential(new Follow("R"+startingLevel+"toRR1", true, mirror));
    addSequential(new AutoVisionDrive(1.5, -0.4, -.25));
    addSequential(new MothOpen());
    addSequential(new PathRRtoLSR(1, mirror));
    addSequential(new AutoVisionDrive(1.5, -0.4, -.25));
    addSequential(new MothClose());
  }
}
