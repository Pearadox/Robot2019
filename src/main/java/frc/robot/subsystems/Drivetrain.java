package frc.robot.subsystems;

import frc.robot.pathfollowing.*;
import frc.robot.*;
import frc.robot.commands.*;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.team319.follower.FollowsArc;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Drivetrain extends Subsystem {
//  /* 
	//VictorSPX is motor controller, but Talon is better and costs more and is more loved
	//To save money, two talons are used as masters and Victors are chained to follow
	//Master motors controls slave motors
	//Numbers represent CAN IDs on CAN bus

	VictorSPX leftSlave1 = new VictorSPX(RobotMap.CANLeftSlave1Victor);
	VictorSPX leftSlave2 = new VictorSPX(RobotMap.CANLeftSlave2Victor);
	VictorSPX rightSlave1 = new VictorSPX(RobotMap.CANRightSlave1Victor);
	VictorSPX rightSlave2 = new VictorSPX(RobotMap.CANRightSlave2Victor);
    TalonSRX leftMaster = new TalonSRX(RobotMap.CANLeftMasterTalon);
	TalonSRX rightMaster = new TalonSRX(RobotMap.CANRightMasterTalon);
// */

	//encoders track wheel rotations (ticks)
	//two ports are used because they have two channels because idk, encoders are digital

	Encoder leftEncoder = new Encoder(RobotMap.leftEncoderA,RobotMap.leftEncoderB);
	Encoder rightEncoder = new Encoder(RobotMap.rightEncoderA, RobotMap.rightEncoderB);

/*
	Victor left1 = new Victor(0);
	Victor left2 = new Victor(1);
	Victor left3 = new Victor(2);
	Victor right1 = new Victor(3);
	Victor right2 = new Victor(4);
	Victor right3 = new Victor(5);
 */

	double lastFeet_r = 0;
	double lastTime = 0;
	double lastVelocity_r = 0;
	double lastAcceleration_r = 0;
	double lastFeet_l = 0;
	double lastVelocity_l = 0;
	double lastAcceleration_l = 0;
	
	public TPoint currentLeftTrajectoryPoint;
	public TPoint currentRightTrajectoryPoint;
	
	public Drivetrain() {
		// /*
		rightSlave1.setInverted(true);
		rightSlave2.setInverted(true);
		rightMaster.setInverted(true);
		leftSlave1.follow(leftMaster);
		leftSlave2.follow(leftMaster);
		rightSlave1.follow(rightMaster);
		rightSlave2.follow(rightMaster);
		leftEncoder.setReverseDirection(true);

		rightMaster.setNeutralMode(NeutralMode.Brake);
		rightSlave1.setNeutralMode(NeutralMode.Brake);
		rightSlave2.setNeutralMode(NeutralMode.Brake);
		leftMaster.setNeutralMode(NeutralMode.Brake);
		leftSlave1.setNeutralMode(NeutralMode.Brake);
		leftSlave2.setNeutralMode(NeutralMode.Brake);
		// */
	}
	
	public void arcadeDrive(double forward, double rotate ) {
		drive(-forward+rotate, -forward-rotate);
	}
	
	public void drive(double leftSpeed, double rightSpeed) {
		setLeft(leftSpeed);
		setRight(rightSpeed);
	}
	
	public void setLeft(double leftSpeed) {
		leftMaster.set(ControlMode.PercentOutput, leftSpeed);
		/*
		left1.set(leftSpeed);
		left2.set(leftSpeed);
		left3.set(leftSpeed);
		*/
	}
	
	public void setRight(double rightSpeed) {
		 rightMaster.set(ControlMode.PercentOutput, rightSpeed);
		 /*
		right1.set(-rightSpeed);
		right2.set(-rightSpeed);
		right3.set(-rightSpeed);
		*/
	}

	public void stop() {
		drive(0, 0);
	}
	
	public long getLeftEncoder() {
		return leftEncoder.get();
		// return 0;
	}
	
	public long getRightEncoder() {
		return rightEncoder.get();
		// return 0;
	}
	
	public double getLeftEncoderInches() {
		return getLeftEncoder()/ RobotMap.ticksPerRev * 2 * Math.PI * 3;
	}
	
	public double getRightEncoderInches() {
		return getRightEncoder()/ RobotMap.ticksPerRev * 2 * Math.PI * 3;
	}

	public double getRightEncoderFeet() {
		return getRightEncoderInches()/12;
	}

	public double getLeftEncoderFeet() {
		return getLeftEncoderInches()/12;
	}

	public double getHeading() {
		return (getLeftEncoder() - getRightEncoder()) / RobotMap.halfTurn*180;
	}
	
	public void zeroEncoders() {
		 leftEncoder.reset();
		 rightEncoder.reset();
	}

	public double getYawEncoder() {
		double initialDifference = Robot.drivetrain.getLeftEncoder() - Robot.drivetrain.getRightEncoder();
		return initialDifference * 360 / 1024;
	}

	public void updateTrajectory(){
		double changeFeet_r = getRightEncoderFeet()-lastFeet_r;
		double changeSeconds = Timer.getFPGATimestamp()-lastTime;
		lastFeet_r = getRightEncoderFeet();
		lastTime = Timer.getFPGATimestamp();
		double velocity_r = changeFeet_r/changeSeconds;
		double changeVelocity_r = lastVelocity_r - lastVelocity_r; 
		double acceleration_r = changeVelocity_r / changeSeconds;
		lastVelocity_r = velocity_r;

		double changeAcceleration_r = acceleration_r - lastAcceleration_r;
		double jerk_r = changeAcceleration_r / changeSeconds;
		lastAcceleration_r = acceleration_r;

		double heading_rad = Math.toRadians(Robot.gyro.getYaw());

		currentRightTrajectoryPoint = new TPoint(getRightEncoderFeet(), velocity_r, acceleration_r, heading_rad);

		double changeFeet = getLeftEncoderFeet()-lastFeet_l;
		lastFeet_l = getLeftEncoderFeet();
		lastTime = Timer.getFPGATimestamp();
		double velocity_l = changeFeet/changeSeconds;
		
		double changeVelocity_l = velocity_l - lastVelocity_l; 
		double acceleration_l = changeVelocity_l / changeSeconds;
		lastVelocity_l = velocity_l;

		double changeAcceleration_l = acceleration_l - lastAcceleration_l;
		double jerk_l = changeAcceleration_l / changeSeconds;
		lastAcceleration_l = acceleration_l;


		currentLeftTrajectoryPoint = new TPoint(getLeftEncoderFeet(), velocity_l, acceleration_l, heading_rad);

	}
	
    public void initDefaultCommand() {
        setDefaultCommand(new DriveWithJoystick());
	}

}