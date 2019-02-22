
package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class IntakeOuttake extends Command {

  boolean hasTimeout;
  double timeout;

  public IntakeOuttake(double timeout) {
    requires(Robot.intake);
    
    if(timeout != -1) {
        this.hasTimeout = true;
        this.timeout = timeout;
    }
  }

  public IntakeOuttake() {
    this(-1);
  }

  @Override
  protected void initialize() {
    if(hasTimeout) setTimeout(timeout);
  }

  @Override
  protected void execute() {
    Robot.intake.set(-1);
  }

  @Override
  protected boolean isFinished() {
    if(hasTimeout) return isTimedOut();
    else return false;
  }

  @Override
  protected void end() {
    Robot.intake.set(0);
  }

  @Override
  protected void interrupted() {
    end();
  }
}
