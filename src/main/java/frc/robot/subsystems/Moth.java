
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Moth extends Subsystem {
  DoubleSolenoid dSolenoid07;
  DoubleSolenoid dSolenoid08;

  public Moth(){
    dSolenoid07 = new DoubleSolenoid(RobotMap.MothDoubleSolenoidForward,RobotMap.MothDoubleSolenoidReverse);
    dSolenoid08 = new DoubleSolenoid(RobotMap.MothDoubleSolenoidForward2,RobotMap.MothDoubleSolenoidReverse2);
  }

  public void open(){
    dSolenoid07.set(DoubleSolenoid.Value.kForward);
    dSolenoid08.set(DoubleSolenoid.Value.kForward);
    
  }

  public void close(){
    dSolenoid07.set(DoubleSolenoid.Value.kReverse);
    dSolenoid08.set(DoubleSolenoid.Value.kReverse);
    
  }
  public void toggle(){
    if (dSolenoid07.get()== Value.kForward ){
      dSolenoid07.set(Value.kReverse);
    }
    else dSolenoid07.set(Value.kForward);
 
  }
    public void toggle2(){
  if (dSolenoid08.get()== Value.kForward ){
    dSolenoid08.set(Value.kReverse);
  }
  else dSolenoid08.set(Value.kForward);
 }
  @Override
  public void initDefaultCommand() {
  }
}
