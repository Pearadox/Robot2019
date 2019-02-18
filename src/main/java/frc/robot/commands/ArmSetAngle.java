/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class ArmSetAngle extends Command {
    double targetAngle;
    double kp = 0.05;
    double ki = 0.0;
    double kd = 0.0001;
    double error, errorSum, lastError;

  public ArmSetAngle(double angle) {
    requires(Robot.arm);

    targetAngle = angle;

    if (!Preferences.getInstance().containsKey("Arm kp")) Preferences.getInstance().putDouble("Arm kp", kp);
    if (!Preferences.getInstance().containsKey("Arm ki")) Preferences.getInstance().putDouble("Arm ki", ki);
    if (!Preferences.getInstance().containsKey("Arm kd")) Preferences.getInstance().putDouble("Arm kd", kd);
  }
  
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    lastError = 0;
    errorSum = 0;

    kp = Robot.prefs.getDouble("Arm kp", kp);
    ki = Robot.prefs.getDouble("Arm ki", ki);
    kd = Robot.prefs.getDouble("Arm kd", kd);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double error = targetAngle - Robot.arm.getRawEncoder();
    // double error = targetAngle - Robot.arm.getAngle();
    double F = Robot.arm.calculateHoldOutput(Robot.arm.getAngle());
    double P = error * kp;
    double I = errorSum * ki;
    double D = (error-lastError) * kd;
    double output = P + I + D + F;

    Robot.arm.setArmSpeed(-output);

    errorSum += error;
    lastError = error;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // double error = targetAngle - Robot.arm.getAngle();
    // if (Math.abs(error) <= 2){
    //   return true;
    // }
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    double output = Robot.arm.calculateHoldOutput(Robot.arm.getAngle()); 
    Robot.arm.setArmSpeed(output);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
