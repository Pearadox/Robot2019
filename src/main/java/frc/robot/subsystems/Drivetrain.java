package frc.robot.subsystems;

import frc.robot.TPoint;
import frc.robot.Robot;
// /*
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Drivetrain extends Subsystem {
	// /*
	VictorSPX leftSlave1 = new VictorSPX(11);
	VictorSPX leftSlave2 = new VictorSPX(10);
	VictorSPX rightSlave1 = new VictorSPX(12);
	VictorSPX rightSlave2 = new VictorSPX(13);
	TalonSRX leftMaster = new TalonSRX(14);
	TalonSRX rightMaster = new TalonSRX(16);

	Encoder leftEncoder = new Encoder(6,7);
	Encoder rightEncoder = new Encoder(8,9);
	// */
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
		
		rightSlave1.setInverted(true);
		rightSlave2.setInverted(true);
		rightMaster.setInverted(true);
		leftSlave1.follow(leftMaster);
		leftSlave2.follow(leftMaster);
		rightSlave1.follow(rightMaster);
		rightSlave2.follow(rightMaster);
		leftEncoder.setReverseDirection(true);
		
	}
	
	public void driveWithJoystick(double forward, double rotate ) {
		setSpeed(-forward+rotate, -forward-rotate);
	}
	
	public void setSpeed(double leftSpeed, double rightSpeed) {
		setSpeedLeft(leftSpeed);
		setSpeedRight(rightSpeed);
	}
	
	public void setSpeedLeft(double leftSpeed) {
		 leftMaster.set(ControlMode.PercentOutput, leftSpeed);
		// left1.set(leftSpeed);
		// left2.set(leftSpeed);
		// left3.set(leftSpeed);
	}
	
	public void setSpeedRight(double rightSpeed) {
		 rightMaster.set(ControlMode.PercentOutput, rightSpeed);
		// right1.set(-rightSpeed);
		// right2.set(-rightSpeed);
		// right3.set(-rightSpeed);
	}

	public void stop() {
		setSpeed(0, 0);
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
		return getLeftEncoder()/ 128.0 * 2 * Math.PI * 3; //256 on practice
	}
	
	public double getRightEncoderInches() {
		return getRightEncoder()/ 128.0 * 2 * Math.PI * 3; //256 on practice
	}

	public double getRightEncoderFeet() {
		return getRightEncoderInches()/12;
	}

	public double getLeftEncoderFeet() {
		return getLeftEncoderInches()/12;
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
		SmartDashboard.putNumber("Velocity_r", velocity_r);
		
		double changeVelocity_r = lastVelocity_r - lastVelocity_r; 
		double acceleration_r = changeVelocity_r / changeSeconds;
		SmartDashboard.putNumber("Acceleration_r", acceleration_r);
		lastVelocity_r = velocity_r;

		double changeAcceleration_r = acceleration_r - lastAcceleration_r;
		double jerk_r = changeAcceleration_r / changeSeconds;
		lastAcceleration_r = acceleration_r;
		SmartDashboard.putNumber("Jerk_r", jerk_r);

		double heading_rad = Math.toRadians(Robot.gyro.getYaw());
		SmartDashboard.putNumber("Heading_rad", heading_rad); 

		currentRightTrajectoryPoint = new TPoint(getRightEncoderFeet(), velocity_r, acceleration_r, heading_rad);

		double changeFeet = getLeftEncoderFeet()-lastFeet_l;
		lastFeet_l = getLeftEncoderFeet();
		lastTime = Timer.getFPGATimestamp();
		double velocity_l = changeFeet/changeSeconds;
		SmartDashboard.putNumber("Velocity_l", velocity_l);
		
		double changeVelocity_l = velocity_l - lastVelocity_l; 
		double acceleration_l = changeVelocity_l / changeSeconds;
		SmartDashboard.putNumber("Acceleration_l", acceleration_l);
		lastVelocity_l = velocity_l;

		double changeAcceleration_l = acceleration_l - lastAcceleration_l;
		double jerk_l = changeAcceleration_l / changeSeconds;
		lastAcceleration_l = acceleration_l;
		SmartDashboard.putNumber("Jerk_l", jerk_l);


		currentLeftTrajectoryPoint = new TPoint(getLeftEncoderFeet(), velocity_l, acceleration_l, heading_rad);

	}
	
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
}

