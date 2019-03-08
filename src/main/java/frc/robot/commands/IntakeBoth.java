
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;


public class IntakeBoth extends Command {

  boolean sensorStop;

  public IntakeBoth(boolean sensorStop) {
    requires(Robot.box);
    requires(Robot.intake);

    this.sensorStop = sensorStop;
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    Robot.box.set(.4);
    Robot.intake.set(.3);
  }

  @Override
  protected boolean isFinished() {
    if(sensorStop && Robot.box.hasBall()) {
      (new DriverRaiseGroup()).start();
      return true;
    }
    else return false;
  }

  @Override
  protected void end() {
    Robot.box.set(0);
    Robot.intake.set(0);
  }

  @Override
  protected void interrupted() {
    end();
  }
}
