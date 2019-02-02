/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.*;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.TPoint;
import frc.robot.TrajectoryGenerator;
import java.util.*;


public class TurnRight extends Command {

  double acceleration = 8;

  double desired_ticks;
  ArrayList<ArrayList<TPoint>> trajectory = new ArrayList();

  public TurnRight(double degrees) {
    this.desired_ticks = degrees / 180. * RobotMap.halfTurn;
    new Thread(new Runnable(){
    
      @Override
      public void run() {
        ArrayList<TPoint> left_trajectory = TrajectoryGenerator.getTrajectory(desired_ticks*RobotMap.feetPerTick, 
            Robot.follower.dt, Robot.follower.maxVelocity, acceleration);
        ArrayList<TPoint> right_trajectory = TrajectoryGenerator.getTrajectory(-desired_ticks*RobotMap.feetPerTick, 
            Robot.follower.dt, Robot.follower.maxVelocity, acceleration);
        trajectory.add(left_trajectory);
        trajectory.add(right_trajectory);
      }
    }).start();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    
    Scheduler.getInstance().add(new Follow(trajectory, true));
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
