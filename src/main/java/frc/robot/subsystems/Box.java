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

public class Box extends Subsystem {
    VictorSPX intakeMotor;
    Ultrasonic ultrasonic;

    public Box(){
        intakeMotor = new VictorSPX(RobotMap.CANBoxIntakeVictor);
        ultrasonic = new Ultrasonic(0, 1);
        ultrasonic.setAutomaticMode(true);
        intakeMotor.setInverted(true);
        intakeMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void set(double percentOutput){
        intakeMotor.set(ControlMode.PercentOutput, percentOutput);    
    }

    public void setHold() {
        set(0.05);
    }

    public double getUltrasonic() {
        return ultrasonic.getRangeInches();
    }

    public boolean hasBall() {
        return ultrasonic.getRangeInches() < 0.4;
    }

    public void initDefaultCommand() {
        setDefaultCommand(new BoxStop());
    }

}
