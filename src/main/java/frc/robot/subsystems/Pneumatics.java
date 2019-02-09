/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Solenoid;

public class Pneumatics extends Subsystem {
  
  Compressor compressor = new Compressor();
   DoubleSolenoid sol07 = new DoubleSolenoid(0, 7);
   DoubleSolenoid sol16 = new DoubleSolenoid(1, 6); 
   DoubleSolenoid sol25 = new DoubleSolenoid(2, 5);
   DoubleSolenoid sol34 = new DoubleSolenoid(3, 4);

  public Pneumatics() {
    compressor.start();
  }

  public void setForward07() {
    sol07.set(DoubleSolenoid.Value.kForward);
  }

  public void setOff07() {
    sol07.set(DoubleSolenoid.Value.kOff);
  }

  public void setReverse07() {
    sol07.set(DoubleSolenoid.Value.kReverse);
  }
  
  public void setForward16() {
    sol16.set(DoubleSolenoid.Value.kForward);
  }

  public void setOff16() {
    sol16.set(DoubleSolenoid.Value.kOff);
  }

  public void setReverse16() {
    sol16.set(DoubleSolenoid.Value.kReverse);
  }
  
  public void setForward25() {
    sol25.set(DoubleSolenoid.Value.kForward);
  }

  public void setOff25() {
    sol25.set(DoubleSolenoid.Value.kOff);
  }

  public void setReverse25() {
    sol25.set(DoubleSolenoid.Value.kReverse);
  }
  
  public void setForward34() {
    sol34.set(DoubleSolenoid.Value.kForward);
  }

  public void setOff34() {
    sol34.set(DoubleSolenoid.Value.kOff);
  }

  public void setReverse34() {
    sol34.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
