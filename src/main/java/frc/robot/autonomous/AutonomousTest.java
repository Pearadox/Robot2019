package frc.robot.autonomous;

import com.team319.follower.FollowArc;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.*;
import frc.robot.commands.*;
import frc.robot.pathfollowing.*;

public class AutonomousTest extends CommandGroup{

    public AutonomousTest() {
        
        // addSequential(new Follow("distanceCalibration", true, false));
        // addSequential(new Follow("turnCalibration", true, false));
    }  

}