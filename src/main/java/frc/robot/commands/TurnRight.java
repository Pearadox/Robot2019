package frc.robot.commands;

import edu.wpi.first.wpilibj.command.*;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.pathfollowing.*;
import java.util.*;


public class TurnRight extends CommandGroup {

  double acceleration = TurnLeft.acceleration;
  ArrayList<ArrayList<TPoint>> trajectory = new ArrayList();

  public TurnRight(double degrees) {
    double desired_ticks = degrees / 180. * RobotMap.halfTurn;

    ArrayList<TPoint> left_trajectory = TrajectoryGenerator.getTrajectory(desired_ticks*RobotMap.feetPerTick, 
        Robot.follower.dt, Robot.follower.maxVelocity, acceleration);
    ArrayList<TPoint> right_trajectory = TrajectoryGenerator.getTrajectory(-desired_ticks*RobotMap.feetPerTick, 
        Robot.follower.dt, Robot.follower.maxVelocity, acceleration);

    trajectory.add(left_trajectory);
    trajectory.add(right_trajectory);
    
    addSequential(new Follow(trajectory, true, false));
  }

}