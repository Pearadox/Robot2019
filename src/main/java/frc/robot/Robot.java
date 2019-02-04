/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.cscore.VideoSink;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.subsystems.*;
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
  public static Pneumatics pneumatics;
  public static PurePursuit pp;

  public static String folder = "/home/lvuser/paths/";
  public static boolean reverseDrivetrain = false;

  UsbCamera camera1, camera2;
  VideoSink server;
  CvSink cvsink1 = new CvSink("cam1cv");
  CvSink cvsink2 = new CvSink("cam2cv");
  boolean prevReverse = false;

  Command autonomousCommand;


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    follower = new Follower();
    pathfinder = new JaciPathfinder();
    drivetrain = new Drivetrain();
    gyro = new IMU();
    prefs = Preferences.getInstance();
    pdp = new PDP();
    limelight = new Limelight();
    pneumatics = new Pneumatics();

    oi = new OI();

    camera1 = CameraServer.getInstance().startAutomaticCapture(0);
    camera2 = CameraServer.getInstance().startAutomaticCapture(1);
    server = CameraServer.getInstance().getServer();

    camera1.setResolution(320, 240);
    camera2.setResolution(320, 240);

    camera1.setPixelFormat(PixelFormat.kMJPEG);
    camera2.setPixelFormat(PixelFormat.kMJPEG);
    
    cvsink1.setSource(camera1);
    cvsink1.setEnabled(true);
    
    cvsink2.setSource(camera2);
    cvsink2.setEnabled(true);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    drivetrain.updateTrajectory();
    SmartDashboard.putNumber("Left Feet Encoder", drivetrain.getLeftEncoderFeet());
    SmartDashboard.putNumber("Right Feet Encoder", drivetrain.getRightEncoderFeet());
    SmartDashboard.putNumber("Voltage", pdp.getVoltage());
    SmartDashboard.putNumber("tx", limelight.getX());
    SmartDashboard.putNumber("ty", limelight.getY());
    SmartDashboard.putBoolean("tv", limelight.targetExists());
    SmartDashboard.putNumber("Limelight Distance", limelight.getDistance());
    SmartDashboard.putNumber("Angle", limelight.getAngle());

    if(!reverseDrivetrain) {
      server.setSource(camera2);
    } else {
      server.setSource(camera1);
    }
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    autonomousCommand = new AutoLeftGroup();
    // autonomousCommand = new DriveForwardCommand(1);

    if (autonomousCommand != null) {
      autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running wheng+*-+
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    reverseDrivetrain = false;
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    
    // if(oi.joystick.getRawButton(7)) pneumatics.setForward07();
    // else if(oi.joystick.getRawButton(8)) pneumatics.setReverse07();

    // if(oi.joystick.getRawButton(9)) pneumatics.setForward16();
    // else if(oi.joystick.getRawButton(10)) pneumatics.setReverse16();

    if(oi.joystick.getRawButton(7)) {
      pneumatics.setForward07();
      pneumatics.setForward16();
      
    }
    else if(oi.joystick.getRawButton(8)) {
      pneumatics.setReverse07();
      pneumatics.setReverse16(); 
    }

    boolean reverseBtn = oi.joystick.getRawButton(2);
    if(reverseBtn && !prevReverse) {
      reverseDrivetrain = !reverseDrivetrain;
    }
    prevReverse = reverseBtn;

  }

  
  //  * This function is called periodically during test mode.
   
  @Override
  public void testPeriodic() {
  }
}
