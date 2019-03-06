/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


/**
 * Add your docs here.
 */
public class Climber extends Subsystem {

  CANSparkMax sparkL, sparkR;
  CANEncoder encoderL, encoderR;

public Climber(){
  sparkL = new CANSparkMax(RobotMap.CANClimberLeftSparkMax, MotorType.kBrushless);
  sparkR = new CANSparkMax(RobotMap.CANClimberRightSparkMax, MotorType.kBrushless);
  encoderL = new CANEncoder(sparkL);
  encoderR = new CANEncoder(sparkR);
  
}

  public void set(double speedL, double speedR){
    sparkL.set(-speedL);
    sparkR.set(speedR);
  }

  public double getLeftRotations() {
    return -encoderL.getPosition();
  }
  
  public double getRightRotations() {
    return encoderR.getPosition();
  }

  public void zero() {
    encoderL.setPosition(0);
    encoderR.setPosition(0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
