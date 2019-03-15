
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class DriverRaiseGroup extends CommandGroup {
  
  public DriverRaiseGroup() {
    
    addSequential(new IntakeStop());
    addSequential(new BoxStop());
    
    addSequential(new IntakeLower(2));
    addSequential(new ArmGoCargo());
    addSequential(new IntakeRaise());
  }
}
