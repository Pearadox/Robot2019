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

public class LSRtoCR extends CommandGroup {

  public LSRtoCR(int cargoTarget, boolean mirror) {
    addSequential(new Follow("LSRtoCR" + cargoTarget, false, mirror));
    // if(!mirror) addSequential(new TurnRight(45, 3));
    // else addSequential(new TurnLeft(45, 3));
    addSequential(new DriveForward(-2));
  }
}
