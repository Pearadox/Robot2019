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
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.Robot;
import frc.robot.RobotMap;


public class DriveWithJoystick extends Command {

  double turnRate = 360;  // deg per sec
  double targetHeading, lastTime, lastError;
  
  double kp = 0.01;
  double kd = 0.2;

  public DriveWithJoystick() {
    requires(Robot.drivetrain);

    if (!Preferences.getInstance().containsKey("GyroDrive kp")){
      Preferences.getInstance().putDouble("GyroDrive kp", kp);
    }
    if (!Preferences.getInstance().containsKey("GyroDrive kd")){
      Preferences.getInstance().putDouble("GyroDrive kd", kd);
    }
  }

  @Override
  protected void initialize() {
    targetHeading = Robot.gyro.getYaw();
    // targetHeading = (Robot.drivetrain.getLeftEncoder() - Robot.drivetrain.getRightEncoder())/RobotMap.halfTurn*180;
    lastTime = Timer.getFPGATimestamp();
    lastError = 0;
    
    kp = Robot.prefs.getDouble("GyroDrive kp", kp);
    kd = Robot.prefs.getDouble("GyroDrive kd", kd);
  }

  @Override
  protected void execute() {
    
    boolean overrideOperator = Robot.oi.operator.getRawButton(1);

    double throttle = overrideOperator ? Robot.oi.operator.getRawAxis(1) : Robot.oi.joystick.getRawAxis(1);
    double twist = overrideOperator ? Robot.oi.operator.getRawAxis(2) : Robot.oi.joystick.getRawAxis(2);
    boolean reverse = Robot.reverseDrivetrain;
    boolean reduce = Robot.oi.joystick.getRawButton(1);

    if(overrideOperator) reduce = true;

    if(Math.abs(throttle)<.15) 
      throttle = 0;
    if(Math.abs(twist)<.25) 
      twist = 0;

    double f = .25;
    throttle = Math.copySign(((throttle * throttle + Math.abs(2*throttle*f)) / (1+2*f)), throttle);
    twist = Math.copySign(((twist * twist + Math.abs(2*twist *f)) / (1+2*f)), twist);

    if(reverse) {
      throttle *= -1;
    }
    if(reduce) {
      throttle *= 0.5;
      twist *= 0.3;
    }

    if(RobotMap.gyroDrive) {
      double dt = Timer.getFPGATimestamp() - lastTime;
      lastTime = Timer.getFPGATimestamp();
      double currentHeading = Robot.gyro.getYaw();
      targetHeading += twist * turnRate * dt;

      double error = targetHeading - currentHeading;

      double P = kp * error;
      double D = kd * (error - lastError);
      lastError = error;

      double output = P + D;
      Robot.drivetrain.arcadeDrive(throttle, output);
    }
    else Robot.drivetrain.arcadeDrive(throttle, twist);
  }

  @Override
  protected boolean isFinished() {
    return false;
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
