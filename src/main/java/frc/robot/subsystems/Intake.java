/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.Solenoid;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;


/**
 * Add your docs here.
 */

public class Intake extends Subsystem {
 
  VictorSPX wheels;
  Solenoid raiserSol1, raiserSol2;

  public Intake(){
    wheels = new VictorSPX(RobotMap.CANIntakeVictor);
    raiserSol1 = new Solenoid(RobotMap.IntakeSolenoid1);
    raiserSol2 = new Solenoid(RobotMap.IntakeSolenoid2);

    raiserSol2.set(raiserSol1.get());
  }


  public void lower(){
    raiserSol1.set(false);
    raiserSol1.set(false);
  }

  public void raise(){
    raiserSol1.set(true);
    raiserSol2.set(true);
  }

  public void toggleRaise(){
    boolean toSet = !raiserSol1.get();
    raiserSol1.set(toSet);
    raiserSol2.set(toSet);
  }
  
  public void set(double setSpeed){
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
