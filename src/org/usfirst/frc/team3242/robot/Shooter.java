/**
 * 
 */
package org.usfirst.frc.team3242.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;


/**
 * Class for control of shooter (using PID).
 * 
 * @author Gabriel Flechas
 *
 */
public class Shooter {
	private boolean isEnabled;
	private WPI_TalonSRX shooter;
	private Spark elevator;
	private Timer elevatorTimer;
	private final double elevatorDelay = 1.5;
	private Encoder encoder;
	private double rps; // is RPS instead of RPM, because encoder returns in distance per second. Setters and getter are 
						// converted to and from RPM for easier human input and reading
	private final double maxRPM = 2059;
	private final double maxRPS = maxRPM*60;
	private PIDController pid;
	private double speedTolerance; // in percentages
	private final double elevatorSpeed = 0.35;

	/**
	 * 
	 * @param shooter Motor controller for motor for shooter
	 * @param encoder Encoder for shooter speed
	 * @param elevator shooter's elevator in the hopper
	 */
	public Shooter(WPI_TalonSRX shooter, Encoder encoder, Spark elevator) {
		this.encoder = encoder;
		this.shooter = shooter;
		this.elevator = elevator;
		this.encoder.setPIDSourceType(PIDSourceType.kRate);
		isEnabled = false;
		pid = new PIDController(0.7, 0.01, 0, 1 / maxRPS, encoder, shooter); //need to configure PID values
		pid.setInputRange(-maxRPS, maxRPS); //need to test
		pid.setOutputRange(-100, 100);
		speedTolerance = 0.1;
		pid.setPercentTolerance(speedTolerance);
		elevatorTimer = new Timer();
		elevatorTimer.start();
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
	
	public double getRPS(){
		return encoder.getRate();
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
		if (isEnabled && (pid.onTarget() || elevatorTimer.get() > elevatorDelay)){ //calibrate range
			elevator.set(-elevatorSpeed); //need to test
		}
		else{
			elevator.set(0);
		}
	}
	/**
	 * Enables PID/shooter
	 */
	public void enable(){
		elevatorTimer.reset();
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
