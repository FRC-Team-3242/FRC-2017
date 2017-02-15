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
	private PIDController pid;
	private double speedTolerance;

	/**
	 * @param Motor controller for motor for shooter
	 */
	public Shooter(CANTalon shooter, Encoder encoder, CANTalon elevator) {
		this.encoder = encoder;
		this.shooter = shooter;
		this.elevator = elevator;
		//40 pulses per rotation for a cimcoder
		this.encoder.setDistancePerPulse(1/40); // so one rotation = one unit for rate
		this.encoder.setPIDSourceType(PIDSourceType.kRate);
		isEnabled = false;
		pid = new PIDController(0.7, 0.01, 0, encoder, shooter); //need to enter PID values
		pid.setPercentTolerance(5);
	}

	
	
	
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
	 * @return the speedTolerance
	 */
	public double getSpeedTolerance() {
		return speedTolerance;
	}

	/**
	 * @param speedTolerance the speedTolerance to set (rpm)
	 */
	public void setSpeedTolerance(double speedTolerance) {
		this.speedTolerance = speedTolerance;
		pid.setAbsoluteTolerance(speedTolerance);
	}

	/**
	 * turns on elevator if the motor is up to speed (in a range of speed tolerance)
	 */
	public void elevate(){
		if (isEnabled && pid.onTarget()){ //calibrate range
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
