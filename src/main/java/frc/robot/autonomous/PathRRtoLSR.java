/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.*;

public class PathRRtoLSR extends CommandGroup {
  /**
   * Add your docs here.
   */
  public PathRRtoLSR(int rocketTarget, boolean mirror) {
    addSequential(new Follow("RR" + rocketTarget + "toLSR", false, mirror));
    addSequential(new TurnAbsolute(180));
    addSequential(new DriveForward(-1));
  }
}
