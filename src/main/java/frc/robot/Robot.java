/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.cscore.VideoSink;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake;
import frc.robot.pathfollowing.*;
import frc.robot.autonomous.*;
import frc.robot.commands.*;



public class Robot extends TimedRobot {
  public static Drivetrain drivetrain;
  public static IMU gyro;
  public static OI oi;
  public static Follower follower;
  public static JaciPathfinder pathfinder;
  public static Preferences prefs;
  public static PDP pdp;
  public static Limelight limelight;
  public static PurePursuit pp;
  public static Intake intake;
  public static Moth moth;
  public static Climber climber;
  public static Arm arm;
  public static Box box;

  Compressor compressor = new Compressor();

  public static String folder = "/home/lvuser/paths/";
  public static boolean reverseDrivetrain = false;
  public static boolean isFollowingPath = false;

  UsbCamera camera1, camera2;
  VideoSink server;
  CvSink cvsink1 = new CvSink("cam1cv");
  CvSink cvsink2 = new CvSink("cam2cv");
  boolean prevReverse = false;

  Command autonomousCommand;

  @Override
  public void robotInit() {
    follower = new Follower();
    pathfinder = new JaciPathfinder();
    pp = new PurePursuit();
    drivetrain = new Drivetrain();
    gyro = new IMU();
    prefs = Preferences.getInstance();
    pdp = new PDP();
    limelight = new Limelight();
    intake = new Intake();
    moth = new Moth();
    climber = new Climber();
    arm = new Arm();
    box = new Box();
    compressor.start();

    oi = new OI();

    if(RobotMap.enableCameras) {
	    camera1 = edu.wpi.first.cameraserver.CameraServer.getInstance().startAutomaticCapture(1);
      camera2 = edu.wpi.first.cameraserver.CameraServer.getInstance().startAutomaticCapture(0);
      // server = edu.wpi.first.cameraserver.CameraServer.getInstance().getServer();      
      // cvsink1.setSource(camera1);
      // cvsink1.setEnabled(true);
      
      // cvsink2.setSource(camera2);
      // cvsink2.setEnabled(true);
    }

    if(arm != null) arm.zero();
    if(gyro != null) gyro.zero();
    if(climber != null) climber.zero();
    if(limelight != null) limelight.lightOff();
    if(intake != null) intake.raise();
    if(moth != null) moth.open();

    SmartDashboard.putData("L1Test", new TestIndividualMotor(1, false, 3));
    SmartDashboard.putData("L2Test", new TestIndividualMotor(2, false, 3));
    SmartDashboard.putData("L3Test", new TestIndividualMotor(3, false, 3));
    SmartDashboard.putData("R1Test", new TestIndividualMotor(1, true, 3));
    SmartDashboard.putData("R2Test", new TestIndividualMotor(2, true, 3));
    SmartDashboard.putData("R3Test", new TestIndividualMotor(3, true, 3));
  }

  
  double start = 0;
  @Override
  public void robotPeriodic() {
    drivetrain.updateTrajectory();
    SmartDashboard.putNumber("Left Encoder", drivetrain.getLeftEncoder());
    SmartDashboard.putNumber("Right Encoder", drivetrain.getRightEncoder());
    SmartDashboard.putNumber("Arm Encoder", arm.getAngle());
    SmartDashboard.putBoolean("Limit Switch", arm.getLimit());
    SmartDashboard.putNumber("Heading", gyro.getYaw());
    SmartDashboard.putNumber("tx", limelight.getX());
    SmartDashboard.putBoolean("tv", limelight.targetExists());
    SmartDashboard.putNumber("Ultrasonic", box.getUltrasonic());
    SmartDashboard.putNumber("ClimberL Enc", climber.getLeftRotations());
    SmartDashboard.putNumber("ClimberR Enc", climber.getRightRotations());
    

    // if(RobotMap.enableCameras) {
    //   if(!reverseDrivetrain) {
    //     server.setSource(camera2);
    //   } else {
    //     server.setSource(camera1);
    //   }
    // }
  }
  
  @Override
  public void disabledInit() {
    if(arm != null) arm.set(0);
    if(intake != null) intake.raise();

    limelight.lightOff();
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
    limelight.lightOn();
  }
  
  @Override
  public void autonomousInit() {

    gyro.zero(180);  // facing backwards

    int switch1 = 1;  //     Side: L, M, R
    int switch2 = 1;  //      Pos: L1, -, L2
    int switch3 = 1;  //  Hatch 1: CargoMid, -, Rocket
    int switch4 = 1;  //  Hatch 2: CargoMid, -, Rocket
    if(oi.autoBox.getRawButton(1)) switch1 = 1;
    else if(oi.autoBox.getRawButton(2)) switch1 = 2;
    else if(oi.autoBox.getRawButton(3)) switch1 = 3;
    if(oi.autoBox.getRawButton(4)) switch2 = 1;
    else if(oi.autoBox.getRawButton(5)) switch2 = 2;
    else if(oi.autoBox.getRawButton(6)) switch2 = 3;
    if(oi.autoBox.getRawButton(7)) switch3 = 1;
    else if(oi.autoBox.getRawButton(8)) switch3 = 2;
    else if(oi.autoBox.getRawButton(9)) switch3 = 3;
    if(oi.autoBox.getRawButton(10)) switch4 = 1;
    else if(oi.autoBox.getRawButton(11)) switch4 = 2;
    else if(oi.autoBox.getRawButton(12)) switch4 = 3;

    if(switch1 == 3 && switch2 == 1 && switch3 == 1 && switch4 == 1)
      autonomousCommand = new AutonomousRtoCMRtoCML(1, false);
    else if(switch1 == 3 && switch2 == 1 && switch3 == 1 && switch4 == 2)
      autonomousCommand = new AutonomousRtoCMRtoRR(1, false);
    else autonomousCommand = new AutonomousDefault();

    autonomousCommand = new AutonomousDefault();
    // autonomousCommand = new AutonomousTest();
    // autonomousCommand = new AutonomousRtoCMRtoCML(1, false);
    // autonomousCommand = new AutonomousRtoCMRtoRR(1, false);
    // autonomousCommand = new AutoVisionDrive(1.5, -0.4, -.25);

    if (autonomousCommand != null) {
      autonomousCommand.start();
    }
  }

  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  public void stopAutonomous() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopInit() {
    stopAutonomous();

    reverseDrivetrain = false;
  }

  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

    boolean reverseBtn = oi.joystick.getRawButton(2);
    if(reverseBtn && !prevReverse) {
      // reverseDrivetrain = !reverseDrivetrain;
    }
    prevReverse = reverseBtn;


    // if(arm.getLimit()) arm.zero();
  }

  @Override
  public void testPeriodic() {
  }
}
