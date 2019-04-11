/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.*;

public class GroupCR1toCR2 extends CommandGroup {
  /**
   * Add your docs here.
   */
  public GroupCR1toCR2(boolean mirror) {
    addSequential(new PathCR1toCR2(mirror, 1));
    addParallel(new DriverLowerGroup(false));
    addSequential(new TurnAbsolute(-30));
  }
}
