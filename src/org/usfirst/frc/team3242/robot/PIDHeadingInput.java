package org.usfirst.frc.team3242.robot;

import com.ctre.PigeonImu;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class PIDHeadingInput implements PIDSource {

	PigeonImu pigeon;
	
	public PIDHeadingInput(PigeonImu pigeon){
		this.pigeon	= pigeon;
	}
	
	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		//do nothing, only use displacement
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return PIDSourceType.kDisplacement;
	}

	@Override
	public double pidGet() {
		//get general status is unnecessary
		//its only used to display info about temperature, boot status, etc. for debugging
		
		//get absolute value of heading [0,360)
		double[] ypr = new double[3];
		pigeon.GetYawPitchRoll(ypr);
		double direction = Math.signum(ypr[0]);
		double yaw = Math.abs(ypr[0]) % 360.0;// -90 + 360
		if(direction < 0){
			yaw = 360.0 - yaw;
		}
		return yaw;
	}

}
