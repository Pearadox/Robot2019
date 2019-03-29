
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;


public class IntakeBoth extends Command {

  boolean sensorStop;
  boolean stopTimerSet;

  public IntakeBoth(boolean sensorStop) {
    requires(Robot.box);
    requires(Robot.intake);

    this.sensorStop = sensorStop;
  }

  @Override
  protected void initialize() {
    stopTimerSet = false;
  }

  @Override
  protected void execute() {
    Robot.box.set(.65);
    Robot.intake.set(.5);
  }

  @Override
  protected boolean isFinished() {
    if(stopTimerSet && isTimedOut()) {
      // (new DriverRaiseGroup()).start();
      return true;
    }
    else if(sensorStop && Robot.box.hasBall() && !stopTimerSet) {
      setTimeout(0.5);
      stopTimerSet = true;
      return false;
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
