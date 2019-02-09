package frc.robot.subsystems;

import frc.robot.pathfollowing.*;
import frc.robot.*;
import frc.robot.commands.*;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends Subsystem {
    double angleMin = 45.;
    double angleMax = 180.;
    double volMin = 3.998;
    double volMax = 1.518;
    AnalogInput potentiometer;
    VictorSPX intakeMotor;
    VictorSPX positionMotor;

    public Arm(){
        potentiometer = new AnalogInput(-1);
        intakeMotor = new VictorSPX(-1);
        positionMotor = new VictorSPX(-1);
    }

    public double getAngle(){
        
        return (potentiometer.getVoltage()-volMin)*(angleMax-angleMin)/(volMax-volMin) + angleMin;

    }

    public void setIntakeSpeed(double percentOutput){
        intakeMotor.set(ControlMode.PercentOutput, percentOutput);    
    }

    public void setArmSpeed(double percentOutput){
        if (getAngle()>180) return;
        positionMotor.set(ControlMode.PercentOutput, percentOutput);
    }

    public void initDefaultCommand() {
        
    }

    public double calculateHoldOutput(double angle){
		double amplitude = 0.138;
		double equation = amplitude * Math.sin(angle*Math.PI/180);
		return equation;
    }



}
