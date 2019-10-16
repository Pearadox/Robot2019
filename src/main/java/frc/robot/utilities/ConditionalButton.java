package frc.robot.utilities;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class ConditionalButton extends JoystickButton {
  private BooleanSupplier conditon;

  public ConditionalButton(GenericHID device, int button, BooleanSupplier condition) {
    super(device, button);
    this.conditon = condition;
  }

  @Override
  public boolean get() {
    if (conditon.getAsBoolean() && super.get()) {
      return true;
    }
    return false;
  }
}