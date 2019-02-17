/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class TurnLeft extends Command {

  double setHeading, timeout;
  double targetHeading;
  double lastTime, lastError;
  double kp = .006;
  double ki = .00002;
  double kd = .05;

  double maxIntegral = 0.2;
  double maxP = 0.8;
  double frictionCompensation = 0.09;

  double errorSum;
  boolean reachedThreshold = false;

  public TurnLeft(double setHeading, double timeout) {
    requires(Robot.drivetrain);
    this.setHeading = setHeading;
    this.timeout = timeout;

    if (!Preferences.getInstance().containsKey("Gyro kp")){
      Preferences.getInstance().putDouble("Gyro kp", kp);
    }
    if (!Preferences.getInstance().containsKey("Gyro ki")){
      Preferences.getInstance().putDouble("Gyro ki", ki);
    }
    if (!Preferences.getInstance().containsKey("Gyro kd")){
      Preferences.getInstance().putDouble("Gyro kd", kd);
    }
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    setTimeout(timeout);
    lastTime = Timer.getFPGATimestamp();
    lastError = 0;
    errorSum = 0;
    this.reachedThreshold = false;

    // targetHeading = Robot.gyro.getYaw() - setHeading;
    targetHeading = Robot.drivetrain.getHeading() - setHeading;

    kp = Robot.prefs.getDouble("Gyro kp", kp);
    ki = Robot.prefs.getDouble("Gyro ki", ki);
    kd = Robot.prefs.getDouble("Gyro kd", kd);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double dt = Timer.getFPGATimestamp() - lastTime;
    lastTime = Timer.getFPGATimestamp();
    double currentHeading = Robot.gyro.getYaw();

    double error = targetHeading - currentHeading;
    errorSum += error;

    double P = kp * error;
    double I = ki * errorSum;
    double D = kd * (error - lastError);
    lastError = error;

    if(Math.abs(P) > maxP) P = Math.copySign(maxP, P);
    if(Math.abs(I) > maxIntegral) I = Math.copySign(maxIntegral, I);

    double output = P + I + D;
    output += Math.copySign(frictionCompensation, output);
    Robot.drivetrain.setSpeed(output, -output);

    SmartDashboard.putNumber("Gyro PID Error", error);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(!reachedThreshold && Math.abs(targetHeading - Robot.gyro.getYaw()) < 3) setTimeout(.3); // give extra time to finish
    if(Math.abs(targetHeading - Robot.gyro.getYaw()) < 1) return true;
    return isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drivetrain.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
