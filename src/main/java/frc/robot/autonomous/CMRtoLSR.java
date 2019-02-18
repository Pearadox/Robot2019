/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.Delay;
import frc.robot.commands.DriveForward;
import frc.robot.commands.Follow;
import frc.robot.commands.TurnAbsolute;
import frc.robot.commands.TurnLeft;
import frc.robot.commands.TurnRight;

public class CMRtoLSR extends CommandGroup {
  /**
   * Add your docs here.
   */
  public CMRtoLSR(boolean mirror) {
    addSequential(new Follow("CMRtoLSR", false, mirror));
    addSequential(new TurnAbsolute(0));
    addSequential(new DriveForward(-1));
    addSequential(new Delay(.5));
  }
}
