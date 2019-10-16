/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.buttons.Trigger;

/**
 * Add your docs here.
 */
public class ConditionalTrigger extends Trigger {
  private GenericHID device;
  private int trigger;
  private double threshold;

  private BooleanSupplier condition;

  @Override
  public boolean get() {
    if (device.getRawAxis(trigger) > threshold && condition.getAsBoolean()) {
      return true;
    }
    return false;
  }

  public ConditionalTrigger(GenericHID device, int trigger, BooleanSupplier condition) {
    this(device, trigger, condition, 0.4);
  }

  public ConditionalTrigger(GenericHID device, int trigger, BooleanSupplier condition, double threshold) {
    this.device = device;
    this.trigger = trigger;
    this.threshold = threshold;
    this.condition = condition;
  }
}
