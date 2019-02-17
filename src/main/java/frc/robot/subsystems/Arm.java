package frc.robot.subsystems;

import frc.robot.pathfollowing.*;
import frc.robot.*;
import frc.robot.commands.*;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends Subsystem {
    double angleMin = 45.;
    VictorSPX intakeMotor;
    CANSparkMax armMotor;
    CANEncoder encoder;

    public Arm(){
        // intakeMotor = new VictorSPX(-1);
        armMotor = new CANSparkMax(3, MotorType.kBrushless);
        encoder = new CANEncoder(armMotor);
    }

    public double getAngle(){
        return encoder.getPosition()*360 + angleMin;
    }

    public void setIntakeSpeed(double percentOutput){
        intakeMotor.set(ControlMode.PercentOutput, percentOutput);    
    }

    public void setArmSpeed(double percentOutput){
        if (getAngle()>180) return;
        armMotor.set(percentOutput);
    }

    public void initDefaultCommand() {
        
    }

    public double calculateHoldOutput(double angle){
		double amplitude = 0.138;
		double equation = amplitude * Math.sin(angle*Math.PI/180);
		return equation;
    }



}
