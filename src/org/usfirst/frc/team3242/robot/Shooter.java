/**
 * 
 */
package org.usfirst.frc.team3242.robot;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;


/**
 * Class for control of shooter (using PID).
 * 
 * @author Gabriel Flechas
 *
 */
public class Shooter {
	private boolean isEnabled;
	private CANTalon shooter;
	private CANTalon elevator;
	private Encoder encoder;
	private double rpm;

	/**
	 * @param Motor controller for motor for shooter
	 */
	public Shooter(CANTalon shooter, Encoder encoder, CANTalon elevator) {
		this.encoder = encoder;
		this.shooter = shooter;
		this.elevator = elevator;
		this.encoder.setDistancePerPulse(1/1440); // so one rotation = one unit for rate
		this.encoder.setPIDSourceType(PIDSourceType.kRate);
		isEnabled = false;
	}

	private PIDController pid = new PIDController(0, 0, 0, encoder, shooter); //need to enter PID values
	
	
	/**
	 * 
	 * @return if it's enabled
	 */
	public boolean isEnabled(){
		return isEnabled;
	}
	
	/**
	 * Sets RPM 
	 * @param rpm 
	 */
	public void setRPM(double rpm){
		this.rpm = rpm / 60;
		pid.setSetpoint(this.rpm);

	}
	/**
	 * 
	 * @return RPM
	 */
	public double getRPM(){
		return encoder.getRate() * 60;
	}
	
	/**
	 * turns on elevator if the motor is up to speed (in a range of +-20)
	 */
	public void elevate(){
		if (isEnabled && (getRPM() >= (rpm - 20) && getRPM() <= (rpm + 20))){ //calibrate range
			elevator.set(0.75); //need to test
		}
		else{
			elevator.set(0);
		}
	}
	/**
	 * Enables PID/shooter
	 */
	public void enable(){
		pid.enable();
		pid.setInputRange(-5310, 5310); //need to test
		pid.setOutputRange(-1, 1);
		isEnabled = true;
	}
	/**
	 * Disables PID/shooter
	 */
	public void disable(){
		pid.disable();
		isEnabled = false;
		
	}
	


}
