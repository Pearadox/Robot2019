package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.*;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.pathfollowing.*;
import java.util.*;


public class TurnRelative extends Command {

  double initialHeading = 0;
  double changeHeading = 0;
  double desiredHeading = 0;
  boolean lastErrorPositive = false; 

  double bangbang = .4;

  public TurnRelative(double degrees) {
    this.changeHeading = degrees;
  }

  public void initialize() {
    this.initialHeading = Robot.gyro.getYaw();
    this.desiredHeading = this.initialHeading + this.changeHeading;
    lastErrorPositive = this.desiredHeading-this.initialHeading > 0;
  }

  public void execute() {
    double error = this.desiredHeading - Robot.gyro.getYaw();
    if(error > 0) {
      Robot.drivetrain.drive(bangbang, -bangbang);
    }
    else {
      Robot.drivetrain.drive(-bangbang, bangbang);
    }
  }

  public boolean isFinished() {
    if(this.desiredHeading - Robot.gyro.getYaw() > 0 ^ this.lastErrorPositive) {
      return true;
    }
    else return false;
  }

  public void end() {
    Robot.drivetrain.drive(0, 0);
  }

  public void interrupted() {
    end();
  }

}