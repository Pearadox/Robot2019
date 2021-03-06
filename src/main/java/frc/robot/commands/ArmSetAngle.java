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

public class ArmSetAngle extends Command {
    double targetAngle;
    double kp = 0.01;
    double ki = 0.00013;
    double kd = 0.05;
    double error, errorSum, lastError;
    boolean climbingMode;

  public ArmSetAngle(double angle) {
    requires(Robot.arm);

    targetAngle = angle;
    angle = Math.max(40, angle);
    angle = Math.min(170, angle);

    if (!Preferences.getInstance().containsKey("Arm kp")) Preferences.getInstance().putDouble("Arm kp", kp);
    if (!Preferences.getInstance().containsKey("Arm ki")) Preferences.getInstance().putDouble("Arm ki", ki);
    if (!Preferences.getInstance().containsKey("Arm kd")) Preferences.getInstance().putDouble("Arm kd", kd);
  }

  public ArmSetAngle(boolean climbingMode) {
    this(ClimbGroup.climbingUpAngle);
    this.climbingMode = climbingMode;
  }
  
  @Override
  protected void initialize() {
    setTimeout(2);
    lastError = 0;
    errorSum = 0;

    kp = Robot.prefs.getDouble("Arm kp", kp);
    ki = Robot.prefs.getDouble("Arm ki", ki);
    kd = Robot.prefs.getDouble("Arm kd", kd);
  }

  @Override
  protected void execute() {
    if(climbingMode) {

      if((Robot.climber.getLeftRotations()+Robot.climber.getRightRotations())/2 > 30) {
        targetAngle = ClimbGroup.fallingDownAngle;
      }
      else targetAngle = ClimbGroup.climbingUpAngle;

    }
    double error = targetAngle - Robot.arm.getAngle();
    double F = 0;
    double P = error * kp;
    double I = errorSum * ki;
    double D = (error-lastError) * kd;
    double output = P + I + D + F;

    Robot.arm.set(output);

    errorSum += error;
    lastError = error;
  }

  @Override
  protected boolean isFinished() {

    if(climbingMode) return false;

    double error = targetAngle - Robot.arm.getAngle();
    if (Math.abs(error) <= 2){
      return true;
    }
    return false;
  }

  @Override
  protected void end() {
    double output = Robot.arm.calculateHoldOutput(Robot.arm.getAngle()); 
    Robot.arm.set(output);
  }

  @Override
  protected void interrupted() {
    end();
  }
}
