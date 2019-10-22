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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake;
import frc.robot.pathfollowing.*;
import frc.robot.autonomous.*;
import frc.robot.commands.*;



public class Robot extends TimedRobot {
  public static Drivetrain drivetrain;
  public static IMU gyro;

  SendableChooser<Controllers> controllerChooser;
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
  public static LaunchpadManager launchpad;

  Compressor compressor = new Compressor();

  public static String folder = "/home/lvuser/paths/";
  public static boolean reverseDrivetrain = false;
  public static boolean isFollowingPath = false;

  UsbCamera camera1, camera2;
  VideoSink server;
  CvSink cvsink1 = new CvSink("cam1cv");
  CvSink cvsink2 = new CvSink("cam2cv");
  boolean prevReverse = false;

  SendableChooser<Command> autoChooser;
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
    launchpad = new LaunchpadManager();

    controllerChooser = new SendableChooser<>();
    controllerChooser.setDefaultOption("Joystick", Controllers.JOYSTICK);
    controllerChooser.addOption("XBox Controller", Controllers.XBOX);
    SmartDashboard.putData("Controller", controllerChooser);

    oi = new OI(controllerChooser.getSelected());

    compressor.start();

    if(RobotMap.enableCameras) {
	    camera1 = edu.wpi.first.cameraserver.CameraServer.getInstance().startAutomaticCapture(1);
      camera2 = edu.wpi.first.cameraserver.CameraServer.getInstance().startAutomaticCapture(0);
    }

    if(arm != null) arm.zero();
    if(gyro != null) gyro.zero();
    if(climber != null) climber.zero();
    if(limelight != null) limelight.lightOff();
    if(intake != null) intake.raise();
    if(moth != null) moth.open();

    SmartDashboard.putData("Test Motors", new TestMotors(1));
    SmartDashboard.putData("Limelight toggle LEDs", new LimelightToggleLight());
    SmartDashboard.putData("Gyro", gyro.navx);
    Preferences.getInstance().putDouble("AutonomousDelay", 0);

    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("Default", new AutonomousDefault());
    autoChooser.addOption("Right L1 --> CMR --> CML", new AutonomousRtoCMRtoCML(1, false));
    autoChooser.addOption("Right L1 --> CMR --> CR", new AutonomousRtoCMRtoCR(1, false));
    autoChooser.addOption("Right L1 --> CMR --> RR", new AutonomousRtoCMRtoRR(1, false));
    autoChooser.addOption("Far Right L1 --> Back Rocket", new AutonomousRtoRRtoRR(1, false));
    autoChooser.addOption("Right L2 --> CMR --> CML", new AutonomousRtoCMRtoCML(2, false));
    autoChooser.addOption("Left L1  --> CML", new AutonomousLtoCML(1, false));
    autoChooser.addOption("Test", new AutonomousTest());
    SmartDashboard.putData("Autonomous Chooser", autoChooser);
  }
  
  double start = 0;
  int brownoutCounter = 0;
  @Override
  public void robotPeriodic() {
    drivetrain.updateTrajectory();
    SmartDashboard.putNumber("Left Encoder", drivetrain.getLeftEncoder());
    SmartDashboard.putNumber("Right Encoder", drivetrain.getRightEncoder());
    SmartDashboard.putNumber("Arm Encoder", arm.getAngle());
    SmartDashboard.putNumber("Heading", gyro.getYaw());
    SmartDashboard.putNumber("tx", limelight.getX());
    SmartDashboard.putBoolean("tv", limelight.targetExists());
    SmartDashboard.putNumber("ClimberL Enc", climber.getLeftRotations());
    SmartDashboard.putNumber("ClimberR Enc", climber.getRightRotations());

    if(DriverStation.getInstance().isBrownedOut()) brownoutCounter++;
    SmartDashboard.putNumber("Brownouts", brownoutCounter);

    launchpad.periodicLoop();
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
    
    launchpad.disabledLoop();
  }
  
  @Override
  public void autonomousInit() {

    gyro.zero(180);  // facing backwards

    // autonomousCommand = new AutonomousDefault(delay);
    // autonomousCommand = new AutonomousTest();
    // autonomousCommand = new AutonomousRtoCMRtoCML(1, false);
    // autonomousCommand = new AutonomousLtoCML(1, false, delay);
    // autonomousCommand = new AutonomousRtoRRtoRR(1, false, delay);

    autonomousCommand = (Command) autoChooser.getSelected();

    if (autonomousCommand != null) {
      autonomousCommand.start();
    }
  }

  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();

    limelight.lightOn();
    
    if(oi.driver.getRawButton(2) || launchpad.btns[0][4]) stopAutonomous();

    launchpad.teleopLoop();
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

    boolean reverseBtn = oi.driver.getRawButton(2);
    if(reverseBtn && !prevReverse) {
      // reverseDrivetrain = !reverseDrivetrain;
    }
    prevReverse = reverseBtn;

    launchpad.teleopLoop();
  }

  @Override
  public void testPeriodic() {
  }
}
