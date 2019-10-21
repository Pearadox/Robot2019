/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.*;

public class GroupRtoCMRtoLSR extends CommandGroup {
  /**
   * Add your docs here.
   */
  public GroupRtoCMRtoLSR(int startingLevel, boolean mirror) {
    addSequential(new MothClose());
    addSequential(new Follow("R"+startingLevel+"toCMR", true, mirror, startingLevel==1 ? .6 : .8));
    // addSequential(new Follow("2019RtoCMR", true, mirror, startingLevel==1 ? .6 : .8));
    //addSequential(new PathR1toCMR(false .7)); not working
    addSequential(new AutoVisionDrive(.1, -0.35, -.2));
    addSequential(new DriveTimed(-.2,-.2,.3));
    addSequential(new Delay(.2));
    addSequential(new MothOpen());
    
    addSequential(new PathCMRtoLSR(mirror, .5));
    addSequential(new AutoVisionDrive(0, -.3, -.2));
    addSequential(new DriveTimed(-.2,-.2,.18));
    addSequential(new Delay(.2));
    addSequential(new MothClose());
    addSequential(new Delay(.2));
    addSequential(new PathLSRtoCML(mirror, .7));
    addSequential(new AutoVisionDrive(0, -0.37, -.2));
    addSequential(new MothOpen());
   
    //addSequential(new DriveTimed(-.2,-.2,.18));
    
  }
}
