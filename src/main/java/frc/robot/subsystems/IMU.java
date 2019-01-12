package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class IMU extends Subsystem {

	AHRS navx;
	double yawOffset = 0;
	
	public IMU () {
		navx = new AHRS(SPI.Port.kMXP);
		navx.zeroYaw();
	}
	
	public double getYaw() {
		return navx.getAngle() - yawOffset;
	}
	
	public void zero() {
		yawOffset += getYaw();
	}
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    
    }
}

