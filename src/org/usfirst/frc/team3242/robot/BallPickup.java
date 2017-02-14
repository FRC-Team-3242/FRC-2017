package org.usfirst.frc.team3242.robot;

import com.ctre.CANTalon;

public class BallPickup{
	
	private CANTalon pickup;
	private CANTalon elevator;
	
	public BallPickup(CANTalon pickup, CANTalon elevator){
		this.pickup = pickup;
		this.elevator = elevator;
	}
	
	public void set(boolean enable){
		double speed = 1;
		if(!enable){
			speed = 0;
		}
		pickup.set(speed);
		elevator.set(speed);
	}
}

