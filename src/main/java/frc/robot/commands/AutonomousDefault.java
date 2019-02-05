package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.pathfollowing.*;

public class AutonomousDefault extends CommandGroup{

    public AutonomousDefault() {
        addSequential(new Follow("LtoML", false, false));
    }  

}