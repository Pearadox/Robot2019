/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  public static int limitSwitchArm = 3;
  
  public static boolean enableCameras = true;
  public static boolean gyroDrive = true;

  public static double ticksPerRev = 256;  //ticks
  public static double wheelDiameter = 6./12.;  //ft
  public static double halfTurn = 1090; //ticks per 180 degrees on EACH side

  public static double feetPerTick = Math.PI * wheelDiameter / ticksPerRev;

  // public static int leftEncoderA = 9;
  // public static int leftEncoderB = 8;
  // public static int rightEncoderA = 7;
  // public static int rightEncoderB = 6;
  public static int leftEncoderA = 7;
  public static int leftEncoderB = 6;
  public static int rightEncoderA = 9;
  public static int rightEncoderB = 8;


  public static int CANLeftSlave1Victor = 11;
  public static int CANLeftSlave2Victor = 10;
  public static int CANRightSlave1Victor = 12;
  public static int CANRightSlave2Victor = 13;
  public static int CANLeftMasterTalon = 14;
  public static int CANRightMasterTalon = 16;

  public static int CANIntakeVictor = 20;

  public static int CANArmBLDCSparkMax = 30;
  public static int CANBoxIntakeVictor = 31;

  public static int CANClimberVictor = 40;

  public static int IntakeSolenoid1 = 2;
  public static int IntakeSolenoid2 = 1;
  public static int MothDoubleSolenoidForward = 0;
  public static int MothDoubleSolenoidReverse = 3;
}
