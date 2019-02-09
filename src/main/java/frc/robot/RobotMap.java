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
  
  public static boolean enableCameras = true;

  public static double ticksPerRev = 128;  //ticks
  public static double wheelDiameter = 6./12.;  //ft
  public static double halfTurn = 256; //ticks per 180 degrees on EACH side

  public static double feetPerTick = Math.PI * wheelDiameter / 128;

  
}
