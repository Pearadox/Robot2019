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
import frc.robot.RobotMap;


public class DriveWithJoystick extends Command {

  double turnRate = 360;  // deg per sec
  double targetHeading, lastTime, lastError;

  double kp = 0.01;
  double kd = 0.07;

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
    // targetHeading = Robot.gyro.getYaw();
    targetHeading = (Robot.drivetrain.getLeftEncoder() - Robot.drivetrain.getRightEncoder())/RobotMap.halfTurn*180;
    lastTime = Timer.getFPGATimestamp();
    lastError = 0;
    
    kp = Robot.prefs.getDouble("GyroDrive kp", kp);
    kd = Robot.prefs.getDouble("GyroDrive kd", kd);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    double throttle = Robot.oi.joystick.getRawAxis(1);
    double twist = Robot.oi.joystick.getRawAxis(2);
    boolean reverse = Robot.reverseDrivetrain;
    boolean reduce = Robot.oi.joystick.getRawButton(1);

    if(Math.abs(throttle)<.15) 
      throttle = 0;
    if(Math.abs(twist)<.15) 
      twist = 0;

    double f = .25;
    throttle = Math.copySign(((throttle * throttle + Math.abs(2*throttle*f)) / (1+2*f)), throttle);
    twist = Math.copySign(((twist * twist + Math.abs(2*twist *f)) / (1+2*f)), twist);

    if(reverse) {
      throttle *= -1;
    }
    if(reduce) {
      throttle *= 0.5;
      twist *= 0.2;
    }

    if(RobotMap.gyroDrive) {
      double dt = Timer.getFPGATimestamp() - lastTime;
      lastTime = Timer.getFPGATimestamp();
      // double currentHeading = Robot.gyro.getYaw();
      double currentHeading = (Robot.drivetrain.getLeftEncoder() - Robot.drivetrain.getRightEncoder())/RobotMap.halfTurn*180;
      targetHeading += twist * turnRate * dt;

      double error = targetHeading - currentHeading;

      double P = kp * error;
      double D = kd * (error - lastError);
      lastError = error;

      double output = P + D;
      Robot.drivetrain.driveWithJoystick(throttle, output);
    }
    else Robot.drivetrain.driveWithJoystick(throttle, twist);
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
