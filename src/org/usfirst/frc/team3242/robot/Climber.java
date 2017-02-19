package org.usfirst.frc.team3242.robot;

import com.ctre.CANTalon;

public class Climber {
	private final double speed = 0.8;
	private CANTalon climberMotor;
	public Climber(CANTalon climb){
		this.climberMotor = climb;
	}
	public void climb(boolean climb, boolean reverse){
		if(climb){
			climberMotor.set(speed);
		}else if(reverse){
			climberMotor.set(-speed);
		}else{
			climberMotor.set(0);
		}
	}
}
