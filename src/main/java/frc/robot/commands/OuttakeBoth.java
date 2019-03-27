
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class OuttakeBoth extends Command {

  public OuttakeBoth() {
    requires(Robot.box);
    requires(Robot.intake);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    Robot.box.set(-0.8);
    Robot.intake.set(-1);
  }

  @Override
  protected boolean isFinished() {
    return false;
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
