
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

/**
 * Add your docs here.
 */
public class Moth extends Subsystem {
  DoubleSolenoid allensol07;

  public Moth(){
    allensol07 = new DoubleSolenoid(0,7);
  }

  

  public void open(){
    allensol07.set(DoubleSolenoid.Value.kForward);
    
  }

  public void close(){
    allensol07.set(DoubleSolenoid.Value.kReverse);
    
  }
  public void toggle(){
    if (allensol07.get()== Value.kForward ){
      allensol07.set(Value.kReverse);
    }
    else allensol07.set(Value.kForward);
  }

  
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
