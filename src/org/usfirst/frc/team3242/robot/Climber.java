package org.usfirst.frc.team3242.robot;

import com.ctre.CANTalon;

public class Climber {
	private final double speed = 1;
	private CANTalon climberMotor;
	public Climber(CANTalon climb){
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
