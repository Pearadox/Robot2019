/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class TestMotors extends Command {

  double time;
  int currentMotor;
  double lastTime;
  boolean pausing;

  public TestMotors(double time) {
    requires(Robot.drivetrain);
    this.time = time;
  }

  @Override
  protected void initialize() {
    lastTime = Timer.getFPGATimestamp();
    currentMotor = 1;
    pausing = false;
  }

  @Override
  protected void execute() {
    if(pausing) {
      Robot.drivetrain.stop();
      if(Timer.getFPGATimestamp() - lastTime > 1) {
        pausing = false;
        lastTime = Timer.getFPGATimestamp();
      }
    }
    else if(Timer.getFPGATimestamp()-lastTime >= 1) {
      Robot.drivetrain.stop();
      currentMotor++;
      lastTime = Timer.getFPGATimestamp();
      pausing = true;
    }
    else {
      Robot.drivetrain.setLeftMotor(currentMotor, 0.3);
      Robot.drivetrain.setRightMotor(currentMotor, 0.3);
    }
  }

  @Override
  protected boolean isFinished() {
    return currentMotor > 3;
  }

  @Override
  protected void end() {
    Robot.drivetrain.stop();
  }

  @Override
  protected void interrupted() {
    end();
  }
}
