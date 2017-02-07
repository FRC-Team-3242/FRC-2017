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
	
	private CANTalon motor;
	private Encoder encoder;

	/**
	 * @param Motor controller for motor for shooter
	 */
	public Shooter(CANTalon motor, Encoder encoder) {
		this.encoder = encoder;
		this.motor = motor;
		this.encoder.setDistancePerPulse(1/1440); // so one rotation = one unit for rate
		this.encoder.setPIDSourceType(PIDSourceType.kRate);
	}

	private PIDController pid = new PIDController(0, 0, 0, encoder, motor); //need to enter PID values
	
	/**
	 * Sets RPM
	 * @param rpm 
	 */
	public void setRPM(double rpm){
		pid.setSetpoint(rpm / 60);
	}
	/**
	 * Enables PID/shooter
	 */
	public void enable(){
		pid.enable();
		pid.setInputRange(-5310, 5310); //need to test
		pid.setOutputRange(-1, 1);
	}
	/**
	 * Disables PID/shooter
	 */
	public void disable(){
		pid.disable();
	}

}
