/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ClimberSet extends Command {

  double kp = 0.01;
  double speed;

  public ClimberSet(double speed, double drivetrainSpeed) {
    requires(Robot.climber);
    requires(Robot.drivetrain);

    this.speed = speed;

    if (!Preferences.getInstance().containsKey("Climb kp")) Preferences.getInstance().putDouble("Climb kp", kp);
  }

  @Override
  protected void initialize() {
    kp = Robot.prefs.getDouble("Climb kp", kp);
  }

  @Override
  protected void execute() {
    double encL = Robot.climber.getLeftRotations();
    double encR = Robot.climber.getRightRotations();

    double average = (encL + encR) / 2;

    double error = encL - encR;

    double P = kp * error;

    double outputL = speed - P;
    double outputR = speed + P;


    if(Robot.oi.operator.getRawButton(11) || Robot.oi.operator.getRawButton(12)) {

      if(speed > 0) {

        if(average >= 67) {
          Robot.climber.set(0, 0);
        } else {
          double output = 0.8 * Math.sin(3.14*average/100-29.85) + .1;
          output = Math.max(output, .25);
          
          Robot.climber.set(output, output);
        }

      }
      else {
        Robot.climber.set(speed, speed);
        Robot.arm.set(0);
      }
    }

    else
      Robot.climber.set(0, 0);
      

    if(speed > 0) {
      Robot.drivetrain.drive(speed, speed);
    }
    else Robot.drivetrain.drive(.0, .0);
  }

  @Override
  protected boolean isFinished() {
    return false;
  }
  
  @Override
  protected void end() {
    Robot.climber.set(0, 0);
    Robot.drivetrain.stop();
  }

  @Override
  protected void interrupted() {
    end();
  }
}
