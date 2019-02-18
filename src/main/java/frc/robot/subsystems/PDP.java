/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class PDP extends Subsystem {

  // PowerDistributionPanel pdp = new PowerDistributionPanel(0);

  public double getVoltage(){
    // return pdp.getVoltage();
    return 0;
  }

public String WhatisAllen(){
    String Allen = "bad at everything";
    return Allen;
  }

public String WhatisJayden(){
    String Jayden = "who?";
    return Jayden;
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
