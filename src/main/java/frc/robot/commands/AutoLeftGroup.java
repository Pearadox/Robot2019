package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class AutoLeftGroup extends CommandGroup{

    public AutoLeftGroup() {
        addSequential(new Follow("LtoML"));
    }

}