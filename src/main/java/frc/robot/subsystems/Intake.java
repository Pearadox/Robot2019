/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Solenoid;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;


/**
 * Add your docs here.
 */

public class Intake extends Subsystem {
 
  VictorSPX wheels;
  Solenoid raiserSol;

  public Intake(){
    // wheels = new VictorSPX(-1);
    raiserSol = new Solenoid(1);
  }


  public void lower(){
    raiserSol.set(false);
  }

  public void raise(){
    raiserSol.set(true);
  }

  public void toggleRaise(){
    raiserSol.set(!raiserSol.get());
  }
  
  public void setSpeed(double setSpeed){
    wheels.set(ControlMode.PercentOutput, setSpeed);
  }

  
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());


  }
}
