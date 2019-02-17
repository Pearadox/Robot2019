/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.Follow;

public class AutonomousRtoRRtoRR extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutonomousRtoRRtoRR(int startingLevel, boolean mirror) {
    addSequential(new Follow("R"+startingLevel+"toRR1", true, mirror));
    addSequential(new RRtoLSR(1, mirror));
    addSequential(new LSRtoRR(2, mirror));
    
    
  }
}