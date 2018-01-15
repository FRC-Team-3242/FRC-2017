package org.usfirst.frc.team3242.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Climber {
	private final double speed = 1;
	private WPI_TalonSRX climberMotor;
	public Climber(WPI_TalonSRX climb){
		this.climberMotor = climb;
	}
	public void climb(boolean climb, boolean reverse){
		if(climb){
			climb(speed);
		}else if(reverse){
			climb(-speed);
		}else{
			climb(0);
		}
	}
	public void climb(double s){
		climberMotor.set(s);
	}
}
