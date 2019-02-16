/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.DriveForward;
import frc.robot.commands.Follow;
import frc.robot.commands.TurnLeft;
import frc.robot.commands.TurnRight;

public class RRtoLSR extends CommandGroup {
  /**
   * Add your docs here.
   */
  public RRtoLSR(int rocketTarget, boolean mirror) {
    addSequential(new Follow("RR" + rocketTarget + "toLSR", false, mirror));
    if(!mirror) addSequential(new TurnLeft(135, 3));
    else addSequential(new TurnRight(135, 3));
    addSequential(new DriveForward(-1));
  }
}
