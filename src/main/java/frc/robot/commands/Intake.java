
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class Intake extends Command {

  boolean sensorStop;

  public Intake(boolean sensorStop) {
    requires(Robot.box);
    requires(Robot.intake);

    sensorStop = this.sensorStop;
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    Robot.box.set(.3);
    Robot.intake.set(1);
  }

  @Override
  protected boolean isFinished() {
    if(sensorStop) return Robot.box.hasBall();
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
