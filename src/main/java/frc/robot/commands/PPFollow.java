/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.PPPoint;
import frc.robot.Robot;

public class PPFollow extends Command {

  double currentX = 0, currentY = 0, startingHeading = 0, lastLeft = 0, lastRight = 0;
  int lastClosestPointIndex = 0;

  ArrayList<PPPoint> trajectory = new ArrayList<>();

  public PPFollow(double x, double y, double headingCorrection) {
    requires(Robot.drivetrain);
    trajectory = Robot.pp.generatePath(x, y, headingCorrection);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    startingHeading = Robot.gyro.getYaw();
    lastLeft = Robot.drivetrain.getLeftEncoderFeet();
    lastRight = Robot.drivetrain.getRightEncoderFeet();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // estimate current odometry
    double relativeHeading = Robot.gyro.getYaw() - startingHeading;
    double deltaLeft = Robot.drivetrain.getLeftEncoderFeet() - lastLeft;
    double deltaRight = Robot.drivetrain.getRightEncoderFeet() - lastRight;
    double distance = (deltaLeft + deltaRight) / 2;
    currentX += distance * Math.cos(Math.toRadians(relativeHeading));
    currentY += distance * Math.sin(Math.toRadians(relativeHeading));

    // calculate closest point
    double smallestDistance = Double.MAX_VALUE;
    int closestPointIndex = lastClosestPointIndex;
    for(int i = lastClosestPointIndex; i < trajectory.size(); i++) {
      if(smallestDistance > trajectory.get(i).calculateDistance(currentX, currentY)) {
        smallestDistance = trajectory.get(i).calculateDistance(currentX, currentY);
        closestPointIndex = i;
      }
    }
    PPPoint closestPoint = trajectory.get(closestPointIndex);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
