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
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Arm extends Subsystem {
    double angleMin = 40.;
    VictorSPX intakeMotor;
    CANSparkMax armMotor;
    CANEncoder encoder;
    Ultrasonic ultrasonic;
    DigitalInput limit;

    public Arm(){
        intakeMotor = new VictorSPX(RobotMap.CANArmIntakeVictor);
        armMotor = new CANSparkMax(3, MotorType.kBrushless);
        encoder = new CANEncoder(armMotor);
        ultrasonic = new Ultrasonic(0, 1);
        limit = new DigitalInput(3);
        ultrasonic.setAutomaticMode(true);
        armMotor.setIdleMode(IdleMode.kBrake);
        armMotor.setInverted(false);
        intakeMotor.setInverted(true);
    }

    public double getAngle(){
        return -getRawEncoder()*3.60 + angleMin;
    }

    public double getRawEncoder() {
        return -encoder.getPosition();
    }

    public void setIntakeSpeed(double percentOutput){
        intakeMotor.set(ControlMode.PercentOutput, percentOutput);    
    }

    public void setArmSpeed(double percentOutput){
        if(getAngle() > 170 && percentOutput > 0) return;
        if(getLimit() && percentOutput < 0) return;
        armMotor.set(percentOutput);
    }

    public double getUltrasonic() {
        return ultrasonic.getRangeInches();
    }

    public void zero() {
        encoder.setPosition(0);
    }

    public boolean getLimit() {
        return !limit.get();
    }

    public void initDefaultCommand() {
        
    }

    public double calculateHoldOutput(double angle){
		double amplitude = 0.025;
		double equation = amplitude * Math.sin(angle*Math.PI/180);
		return equation;
    }



}
