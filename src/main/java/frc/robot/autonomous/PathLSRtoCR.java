/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.*;

public class PathLSRtoCR extends CommandGroup {

  public PathLSRtoCR(int cargoTarget, boolean mirror) {
    addSequential(new Follow("LSRtoCR" + cargoTarget, false, mirror));
    addSequential(new TurnAbsolute(mirror ? -90 : 90));
    addSequential(new DriveForward(-2));
  }
}
