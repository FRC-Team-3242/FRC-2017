package org.usfirst.frc.team3242.robot;

import com.ctre.CANTalon;

public class BallPickup{
	
	private CANTalon pickup;
	private CANTalon elevator;
	private final double speed = 0.37;
	private final double speedEleScalar = 1.75;
	private final double reverseSpeed = 0.7;
	
	public BallPickup(CANTalon pickup, CANTalon elevator){
		this.pickup = pickup;
		this.elevator = elevator;
		this.elevator.setInverted(true);
		this.pickup.setInverted(true);
	}
	
	public void set(boolean enable){
		set(enable, false);
	}
	
	public void set(boolean up, boolean down){
		if(up){
			set(speed);
		}else if(down){
			set(-reverseSpeed);
		}else{
			set(0);
		}
	}
	
	public void set(double s){
		pickup.set(s);
		elevator.set(s * speedEleScalar);
	}
}

