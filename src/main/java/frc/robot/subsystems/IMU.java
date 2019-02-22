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
		zero(0);
	}

	public void zero(double extraOffset) {
		yawOffset += getYaw() - extraOffset;
	}

    public void initDefaultCommand() {
    
    }
}

