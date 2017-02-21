/**
 * 
 */
package org.usfirst.frc.team3242.robot;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Spark;


/**
 * Class for control of shooter (using PID).
 * 
 * @author Gabriel Flechas
 *
 */
public class Shooter {
	private boolean isEnabled;
	private CANTalon shooter;
	private Spark elevator;
	private Encoder encoder;
	private double rps; // is RPS instead of RPM, because encoder returns in distance per second. Setters and getter are 
						// converted to and from RPM for easier human input and reading
	private final double maxRPS = 5310;
	private PIDController pid;
	private double speedTolerance; // in percentages

	/**
	 * 
	 * @param shooter Motor controller for motor for shooter
	 * @param encoder Encoder for shooter speed
	 * @param elevator shooter's elevator in the hopper
	 */
	public Shooter(CANTalon shooter, Encoder encoder, Spark elevator) {
		this.encoder = encoder;
		this.shooter = shooter;
		this.elevator = elevator;
		//40 pulses per rotation for a cimcoder
		this.encoder.setDistancePerPulse(1/40); // so one rotation = one unit for rate
		this.encoder.setPIDSourceType(PIDSourceType.kRate);
		isEnabled = false;
		pid = new PIDController(0.7, 0.01, 0, 1 / maxRPS, encoder, shooter); //need to configure PID values
		pid.setInputRange(-maxRPS, maxRPS); //need to test
		pid.setOutputRange(-100, 100);
		speedTolerance = 5;
		pid.setPercentTolerance(speedTolerance);
		shooter.setSafetyEnabled(false);
	}

	public void overrideShooter(double s){
		shooter.set(s);
	}
	
	public void overrideElevator(double s){
		elevator.set(s);
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
		this.rps = rpm / 60;
		pid.setSetpoint(this.rps);
	}
	/**
	 * 
	 * @return RPM
	 */
	public double getRPM(){
		return encoder.getRate() * 60;
	}
	
	public double getdist(){
		return encoder.getDistance();
	}
	
	/**
	 * @return the speedTolerance (percent)
	 */
	public double getSpeedTolerance() {
		return speedTolerance;
	}

	/**
	 * Default speedTolerance is 5%
	 * @param speedTolerance the speedTolerance to set (percentage of tolerance)
	 */
	public void setSpeedTolerance(double speedTolerance) {
		this.speedTolerance = speedTolerance;
		pid.setPercentTolerance(speedTolerance);
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
