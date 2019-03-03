
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class BoxStop extends Command {

  public BoxStop() {
    requires(Robot.box);
  }

  @Override
  protected void initialize() {
    Robot.box.set(0);
  }

  @Override
  protected void execute() {
    }

  @Override
  protected boolean isFinished() {
    return true;
  }

  @Override
  protected void end() {
    Robot.box.set(0);
  }

  @Override
  protected void interrupted() {
    end();
  }
}
