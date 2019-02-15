package frc.robot.autonomous;

import com.team319.follower.FollowArc;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.arcs.DistanceScalingArc;
import frc.arcs.R2toCMRArc;
import frc.arcs.TurnScalingArc;
import frc.robot.*;
import frc.robot.commands.*;
import frc.robot.pathfollowing.*;

public class AutonomousTest extends CommandGroup{

    public AutonomousTest() {
        
        // addSequential(new Follow("distanceCalibration", false, false));
        // addSequential(new Follow("turnCalibration", false, false));

                    // bobtrajectory tuning
        // addSequential(new FollowArc(Robot.drivetrain, new R2toCMRArc()));
        // addSequential(new FollowArc(Robot.drivetrain, new DistanceScalingArc()));
        // addSequential(new FollowArc(Robot.drivetrain, new TurnScalingArc()));
    }  

}