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


public class Drivetrain extends Subsystem implements FollowsArc{
	//  /*
	 
	//victorspx is motor controller, but talon is better and costs more and is more loved
	//to save money, two talons are used as masters and victors are chained to follow
	//master motors controls slave motors
	//numbers represent ports on roborio
	//this is all on CAN bus (bus is connection to different

	VictorSPX leftSlave1 = new VictorSPX(RobotMap.CANLeftSlave1Victor);
	VictorSPX leftSlave2 = new VictorSPX(RobotMap.CANLeftSlave2Victor);
	VictorSPX rightSlave1 = new VictorSPX(RobotMap.CANRightSlave1Victor);
	VictorSPX rightSlave2 = new VictorSPX(RobotMap.CANRightSlave2Victor);
    TalonSRX leftMaster = new TalonSRX(RobotMap.CANLeftMasterTalon);
	TalonSRX rightMaster = new TalonSRX(RobotMap.CANRightMasterTalon);

	//encoders track wheel rotations (track ticks)
	//two ports are used because they have two channels because idk
	//encoders are digital

	Encoder leftEncoder = new Encoder(RobotMap.leftEncoderA,RobotMap.leftEncoderB);
	Encoder rightEncoder = new Encoder(RobotMap.rightEncoderA, RobotMap.rightEncoderB);
	// */
/*
	Victor left1 = new Victor(0);
	Victor left2 = new Victor(1);
	Victor left3 = new Victor(2);
	Victor right1 = new Victor(3);
	Victor right2 = new Victor(4);
	Victor right3 = new Victor(5);
 */

 /*





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

		/*
		// bobtrajectory talon configurations BOB
		leftMaster.setSensorPhase(true);
		rightMaster.setSensorPhase(true);
		leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
		leftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 0);
		rightMaster.configRemoteFeedbackFilter(leftMaster.getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor, 0, 0);
		// rightMaster.configRemoteFeedbackFilter(pigeon.getDeviceID(), RemoteSensorSource.GadgeteerPigeon_Yaw, 1, 0);
		rightMaster.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, 0);
		rightMaster.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.QuadEncoder, 0);
		rightMaster.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, 0, 0);
		rightMaster.configSelectedFeedbackCoefficient(0.5, 0, 0);
		// rightMaster.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor1, 1, 0);
		// rightMaster.configSelectedFeedbackCoefficient((3600.0 / 8192.0), 1, 0);

		double kf = 4;
		double kp = 1.6;
		double ki = 0.00;
		double kd = 4;

		leftMaster.config_kF(0, kf, 0);
		leftMaster.config_kP(0, kp, 0);
		leftMaster.config_kI(0, ki, 0);
		leftMaster.config_kD(0, kd, 0);
		rightMaster.config_kF(0, kf, 0);
		rightMaster.config_kP(0, kp, 0);
		rightMaster.config_kI(0, ki, 0);
		rightMaster.config_kD(0, kd, 0);
		// */
	}
	
	public void driveWithJoystick(double forward, double rotate ) {
		setSpeed(-forward+rotate, -forward-rotate);
	}

	public void stopMotionMagic() {
		// leftMaster.set(ControlMode.PercentOutput, 0);
		// rightMaster.set(ControlMode.PercentOutput, 0);
	}

	public void setMotionMagic(double rotations) {
		// double target = rotations * 128;
		// leftMaster.set(ControlMode.MotionMagic, 1280);
		// rightMaster.set(ControlMode.MotionMagic, 1280);
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
		return getLeftEncoder()/ RobotMap.ticksPerRev * 2 * Math.PI * 3; //256 on practice
	}
	
	public double getRightEncoderInches() {
		return getRightEncoder()/ RobotMap.ticksPerRev * 2 * Math.PI * 3; //256 on practice
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
        setDefaultCommand(new DriveWithJoystick());
	}
	
	  // This should return your left talon object
	  @Override
	  public TalonSRX getLeft() {
		// return leftMaster; 
		return null;
	  }
	
	  // This should return your right talon object
	  @Override
	  
	  public TalonSRX getRight() {
		// return rightMaster; 
		return null;
	  }
	
	  // This should return the current value of your sum sensor that will be configured in a future step
	  @Override
	  public double getDistance() {
		// return rightMaster.getSelectedSensorPosition(0);
		return 0;
	  }
	  
	  // This should return the instance of your drive train
	  @Override
	  public Subsystem getRequiredSubsystem() {
		return Robot.drivetrain;
	  }

}

