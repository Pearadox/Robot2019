
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class BoxIntakeHold extends Command {

  public BoxIntakeHold() {
    requires(Robot.box);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    Robot.box.setHold();
    }

  @Override
  protected boolean isFinished() {
    return false;
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
