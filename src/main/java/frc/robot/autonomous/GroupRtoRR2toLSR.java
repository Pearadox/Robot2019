/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.*;

public class GroupRtoRR2toLSR extends CommandGroup {
  /**
   * Add your docs here.
   */
  public GroupRtoRR2toLSR(int startingLevel, boolean mirror) {
    addSequential(new MothClose());
    addSequential(new Follow("R"+startingLevel+"toRR21of2", false, mirror));
    addSequential(new Follow("R"+startingLevel+"toRR22of2", true, mirror, 1));
    addSequential(new AutoVisionDrive(1.5, -0.35, -.25));
    addSequential(new MothOpen());
    
    addSequential(new PathRRtoLSR(2, mirror));
    addSequential(new AutoVisionDrive(2, -0.3, -.2));
    addSequential(new MothClose());
  }
}
