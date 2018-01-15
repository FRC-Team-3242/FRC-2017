package org.usfirst.frc.team3242.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class BallPickup{
	
	private WPI_TalonSRX pickup;
	private WPI_TalonSRX elevator;
	private final double speed = 0.39;
	private final double speedEleScalar = 1.75;
	private final double reverseSpeed = 0.5;
	
	public BallPickup(WPI_TalonSRX pickup, WPI_TalonSRX elevator){
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

