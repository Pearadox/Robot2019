/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.pathfollowing.TPoint;
import frc.robot.pathfollowing.TrajectoryGenerator;

public class DriveForward extends CommandGroup {
  
  double velocity = Robot.follower.maxVelocity;
  double acceleration = 10;

  public DriveForward(double distance) {
    
    ArrayList<TPoint> sideTraj = TrajectoryGenerator.getTrajectory(distance, Robot.follower.dt, 
                                    velocity, acceleration);  
    ArrayList<ArrayList<TPoint>> trajectory = new ArrayList<>();
    trajectory.add(sideTraj);
    trajectory.add(sideTraj);
    addSequential(new Follow(trajectory, false, true));

  }
}
