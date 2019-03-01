
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class OuttakeGroup extends Command {

  boolean sensorStop;

  public OuttakeGroup() {
    requires(Robot.box);
    requires(Robot.intake);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    Robot.box.set(-1);
    Robot.intake.set(-0.3);
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
