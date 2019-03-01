package frc.robot.subsystems;

import frc.robot.pathfollowing.*;
import frc.robot.*;
import frc.robot.commands.*;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
    CANSparkMax armMotor;
    CANEncoder encoder;
    DigitalInput limit;

    public Arm(){
        armMotor = new CANSparkMax(RobotMap.CANArmBLDCSparkMax, MotorType.kBrushless);
        encoder = new CANEncoder(armMotor);
        limit = new DigitalInput(3);
        armMotor.setIdleMode(IdleMode.kBrake);
        armMotor.setInverted(false);
    }

    public double getAngle(){
        return -getRawEncoder()*3.60 + angleMin;
    }

    public double getRawEncoder() {
        return -encoder.getPosition();
    }

    public void set(double percentOutput){
        if(getAngle() > 170 && percentOutput > 0) return;
        if(getLimit() && percentOutput < 0) return;
        armMotor.set(percentOutput);
    }

    public void zero() {
        encoder.setPosition(0);
    }

    public boolean getLimit() {
        return !limit.get();
    }

    public double calculateHoldOutput(double angle){
		double amplitude = 0.025;
		double equation = amplitude * Math.sin(angle*Math.PI/180);
		return equation;
    }

    public void initDefaultCommand() {
        setDefaultCommand(new ArmHold());
    }
}
