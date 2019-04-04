/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.*;

public class GroupRtoCMRtoLSR extends CommandGroup {
  /**
   * Add your docs here.
   */
  public GroupRtoCMRtoLSR(int startingLevel, boolean mirror) {
    addSequential(new MothClose());
    addSequential(new Follow("R"+startingLevel+"toCMR", true, mirror, startingLevel==1 ? .6 : .8));
    addSequential(new AutoVisionDrive(2.0, -0.4, -.25));
    addSequential(new MothOpen());
    
    addSequential(new PathCMRtoLSR(mirror, .5));
    addSequential(new AutoVisionDrive(1.5, -.2, -.2));
    addSequential(new MothClose());
    addSequential(new Delay(.3));
  }
}
