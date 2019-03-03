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
    // climber = new Climber();
    arm = new Arm();
    box = new Box();
    compressor.start();

    oi = new OI();

    if(RobotMap.enableCameras) {
      camera1 = CameraServer.getInstance().startAutomaticCapture(1);
      camera2 = CameraServer.getInstance().startAutomaticCapture(0);
      server = CameraServer.getInstance().getServer();
      
      cvsink1.setSource(camera1);
      cvsink1.setEnabled(true);
      
      cvsink2.setSource(camera2);
      cvsink2.setEnabled(true);
    }

    if(arm != null) arm.zero();
    if(gyro != null) gyro.zero();
    limelight.lightOff();
  }

  
  double start = 0;
  @Override
  public void robotPeriodic() {
    drivetrain.updateTrajectory();
    SmartDashboard.putNumber("Left Feet Encoder", drivetrain.getLeftEncoderFeet());
    SmartDashboard.putNumber("Right Feet Encoder", drivetrain.getRightEncoderFeet());
    SmartDashboard.putNumber("Left Encoder", drivetrain.getLeftEncoder());
    SmartDashboard.putNumber("Right Encoder", drivetrain.getRightEncoder());
    SmartDashboard.putNumber("Arm Encoder", arm.getAngle());
    SmartDashboard.putNumber("Arm Raw Encoder", arm.getRawEncoder());
    SmartDashboard.putBoolean("Limit Switch", arm.getLimit());
    SmartDashboard.putNumber("Heading", gyro.getYaw());
    SmartDashboard.putNumber("Drivetrain Heading", drivetrain.getHeading());
    SmartDashboard.putNumber("tx", limelight.getX());
    SmartDashboard.putNumber("ty", limelight.getY());
    SmartDashboard.putBoolean("tv", limelight.targetExists());
    SmartDashboard.putNumber("Limelight Distance", limelight.getDistance());
    SmartDashboard.putNumber("Ultrasonic", box.getUltrasonic());
    

    if(RobotMap.enableCameras) {
      if(!reverseDrivetrain) {
        server.setSource(camera2);
      } else {
        server.setSource(camera1);
      }
    }
  }
  
  @Override
  public void disabledInit() {
    if(arm != null) arm.set(0);
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
    limelight.lightOff();
  }
  
  @Override
  public void autonomousInit() {

    gyro.zero(180);  // facing backwards

    // autonomousCommand = new AutonomousTest();
    // autonomousCommand = new AutonomousRtoCMRtoCML(1, false);
    // autonomousCommand = new AutonomousRtoCMRtoCR(1, 1, false);

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
      reverseDrivetrain = !reverseDrivetrain;
    }
    prevReverse = reverseBtn;

    limelight.lightOn();

    // if(arm.getLimit()) arm.zero();
  }

  @Override
  public void testPeriodic() {
  }
}
