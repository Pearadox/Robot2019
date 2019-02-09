/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class ArmSetAngle extends Command {
    double targetAngle = 0.;
    double armkP = 0.1;
    double armkI = 0.0;
    double armkD = 0.;
    double error;
    double errorSum = 0;
    double lastError = 0;

  public ArmSetAngle(double angle) {
      targetAngle = angle;
    // Use requires() here to declare subsystem dependencies
    requires(Robot.arm);
  }
  
  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    lastError = 0;
    errorSum = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double error = targetAngle - Robot.arm.getAngle();
    double F = Robot.arm.calculateHoldOutput(Robot.arm.getAngle());
    double P = error * armkP;
    double I = errorSum * armkI;
    double D = (error-lastError) * armkD;
    double output = P + I + D + F;
    Robot.arm.setArmSpeed(output);
    errorSum += error;
    lastError = error;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    double error = targetAngle - Robot.arm.getAngle();
    if (Math.abs(error) <= 2){
      return true;
    }
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
  }
}
