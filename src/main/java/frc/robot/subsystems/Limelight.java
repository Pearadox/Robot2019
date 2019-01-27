

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.*;
import java.util.*;
import frc.robot.*;


public class Limelight extends Subsystem {


  @Override
  public void initDefaultCommand() {

  }

  public boolean targetExists() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tv = table.getEntry("tv");

    return tv.getDouble(0.0) == 1.0;
  }
  
  public double getX() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");

    return tx.getDouble(0.0);
  }

   public double getY() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = table.getEntry("ty");

    return ty.getDouble(0.0);
  }

   public double getArea() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ta = table.getEntry("ta");

    return ta.getDouble(0.0);
  }

  public double getDistance() {
    return (32.06125-28.5) / Math.tan(Math.toRadians(16.9 - getY()));
  }

  public double getAngle() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry thor = table.getEntry("thor");
    NetworkTableEntry tvert = table.getEntry("tvert");
    double currentRatio = thor.getDouble(0.0) / tvert.getDouble(0.0);
    // double originalRatio = 15.27 / 6.125;
    double originalRatio = 77./34;

    return Math.toDegrees(Math.acos(currentRatio / originalRatio));
  }

  public ArrayList<ArrayList<TPoint>> getTrajectory() {
    double theta = Math.toRadians(90 - getAngle());
    double headingCorrection = Math.toRadians(90-getAngle()-getX());
    double forward = getDistance() * Math.sin(theta);
    double sideways = getDistance() * Math.cos(theta);
    SmartDashboard.putNumber("forward", forward);
    SmartDashboard.putNumber("sideways", sideways);

    return Robot.pathfinder.createShortPath(forward, sideways, headingCorrection);
  }

}