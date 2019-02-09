/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
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

/**
 * An example command.  You can replace me with your own command.
 */
public class DriveWithJoystick extends Command {

  double turnRate = 210;  // deg per sec
  double targetHeading, lastTime, lastError;

  double kp = 0.01;
  double kd = 0.1;

  public DriveWithJoystick() {
    requires(Robot.drivetrain);

    if (!Preferences.getInstance().containsKey("GyroDrive kp")){
      Preferences.getInstance().putDouble("GyroDrive kp", kp);
    }
    if (!Preferences.getInstance().containsKey("GyroDrive kd")){
      Preferences.getInstance().putDouble("GyroDrive kd", kd);
    }
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    targetHeading = Robot.gyro.getYaw();
    lastTime = Timer.getFPGATimestamp();
    lastError = 0;
    
    kp = Robot.prefs.getDouble("GyroDrive kp", kp);
    kd = Robot.prefs.getDouble("GyroDrive kd", kd);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    double forwardJoystick = Robot.oi.joystick.getRawAxis(1);
    double rotateJoystick = Robot.oi.joystick.getRawAxis(2);
    boolean reverse = Robot.reverseDrivetrain;
    boolean reduce = Robot.oi.joystick.getRawButton(1);

    if(Math.abs(forwardJoystick)<.15) {
			forwardJoystick = 0;
		}
    if(reverse) {
      forwardJoystick *= -1;
    }
    if(reduce) {
      forwardJoystick *= 0.5;
      rotateJoystick *= 0.2;
    }

    double dt = Timer.getFPGATimestamp() - lastTime;
    lastTime = Timer.getFPGATimestamp();
    double currentHeading = Robot.gyro.getYaw();
    targetHeading += rotateJoystick * turnRate * dt;

    double error = targetHeading - currentHeading;

    double P = kp * error;
    double D = kd * (error - lastError);
    lastError = error;

    double output = P + D;

    // Robot.drivetrain.driveWithJoystick(forwardJoystick, rotateJoystick);
    Robot.drivetrain.driveWithJoystick(forwardJoystick, output);
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
