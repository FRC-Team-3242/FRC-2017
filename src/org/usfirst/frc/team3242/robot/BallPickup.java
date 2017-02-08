package org.usfirst.frc.team3242.robot;

import com.ctre.CANTalon;

public class BallPickup{
	
	private CANTalon pickup;
	private CANTalon elevator;
	private boolean isEnabled;
	
	public BallPickup(CANTalon pickup, CANTalon elevator){
		this.pickup = pickup;
		this.elevator = elevator;
		isEnabled = false;
	}
	
	public boolean isEnabled(){
		return isEnabled;
	}
	
	public void enable(){
		pickup.set(1);
		elevator.set(1); //Need to test speeds
		isEnabled = true;
	}
	
	public void disable(){
		pickup.set(0);
		elevator.set(0);
		isEnabled = false;
	}
}

