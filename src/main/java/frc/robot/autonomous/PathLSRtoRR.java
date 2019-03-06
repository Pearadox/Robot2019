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
import frc.robot.commands.*;

public class PathLSRtoRR extends CommandGroup {
  /**
   * Add your docs here.
   */
  public PathLSRtoRR(int rocketTarget, boolean mirror) {
    addSequential(new Follow("LSRtoRR" + rocketTarget, false, mirror));
    addSequential(new TurnAbsolute(mirror ? 30 : -30));
    addSequential(new DriveForward(-1));
  }
}
