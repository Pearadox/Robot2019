/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.*;

public class AutonomousLtoCML extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutonomousLtoCML(int startingLevel, boolean mirror) {
    addSequential(new AutoDelay());
    
    addSequential(new MothClose());
    addSequential(new Follow("R"+startingLevel+"toCMR", true, mirror, startingLevel==1 ? .6 : .8));
    addSequential(new AutoVisionDrive(0, -0.35, -.2));
    addSequential(new DriveTimed(-.2,-.2,.18));
    addSequential(new MothOpen());
    
    addSequential(new PathCMLtoLSL(mirror, .5));
    addSequential(new AutoVisionDrive(0, -0.35, -.2));
    addSequential(new DriveTimed(-.2,-.2,.18));
    addSequential(new MothOpen());
    addSequential(new Delay(.3));

    addSequential(new GroupLSRtoCML(true));
  }
}
