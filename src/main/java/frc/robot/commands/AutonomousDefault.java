package frc.robot.commands;

import com.team319.follower.FollowArc;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.arcs.DistanceScalingArc;
import frc.arcs.TurnScalingArc;
import frc.robot.Robot;
import frc.robot.pathfollowing.*;

public class AutonomousDefault extends CommandGroup{

    public AutonomousDefault() {
        addSequential(new Follow("LtoML", false, false));

                    // bobtrajectory tuning
        // addSequential(new FollowArc(Robot.drivetrain, new DistanceScalingArc()));
        addSequential(new FollowArc(Robot.drivetrain, new TurnScalingArc()));
    }  

}