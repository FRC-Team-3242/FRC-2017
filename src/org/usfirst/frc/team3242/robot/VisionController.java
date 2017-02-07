package org.usfirst.frc.team3242.robot;

import com.ctre.PigeonImu;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;

public class VisionController {
	
	private VisionServer vision;
	private PigeonImu gyro;
	private RobotDrive drive;
	
	//both tolerances within 0.5%
	private final double xTolerance = 6.4;
	private final double yTolerance = 3.6;
	private final double gyroTolerance = 1.8;
	
	private final double xMax = 1280;
	private final double yMax = 720;
	
	//center point of ideal lift
	private final double xLift = 400;
	private final double yLift = 600;
	
	//ideal angles for lifts
	private final double angleA = 0;
	private final double angleB = 120;
	private final double angleC = 240;
	
	//center point of boiler
	private final double xBoiler = 700;
	private final double yBoiler = 100;
	
	/**
	 * integer for state machine
	 * 0-99		: nothing
	 * 100-199	: lift
	 * 200-299	: boiler
	 */
	private int autoState;
	private double closestIdealAngle;
	private Timer autoTimer;
	
	public VisionController(VisionServer vision, RobotDrive drive, PigeonImu gyro){
		this.vision = vision;
		this.drive = drive;
		autoState = 0;
		closestIdealAngle = 0;
		autoTimer = new Timer();
		autoTimer.start();
		this.gyro = gyro;
	}
	
	public void startLiftTracking(){
		autoState = 100;
	}
	
	public void startBoilerTracking(){
		autoState = 200;
	}
	
	public void stopAll(){
		autoState = 0;
	}
	
	public int getAutoState(){
		return autoState;
	}
	
	private boolean linedUpToLiftX(){
		return numberInTolerance(vision.getX(),xLift,xTolerance);
	}
	private boolean linedUpToLiftY(){
		return numberInTolerance(vision.getY(),yLift,yTolerance);
	}

	public boolean linedUpToBoiler(){
		return linedUpToBoilerX() && linedUpToBoilerY();
	}
	private boolean linedUpToBoilerX(){
		return numberInTolerance(vision.getX(),xBoiler,xTolerance);
	}
	private boolean linedUpToBoilerY(){
		return numberInTolerance(vision.getY(),yBoiler,yTolerance);
	}
	
	/**
	 * should be called ONCE per iteration at end of iteration
	 * for instance, at the end of the teleopPeriodic function
	 * 
	 * this function executes the controller's state machine
	 */
	public void update(){
		switch(autoState){
		case(0):
			//do nothing
			break;
		
		case(100):
			//start lift sequence, find closest ideal angle
			
			break;
		
		case(200):
			//start boiler sequence, center horizontally
			if(!linedUpToBoilerX()){
				double x = pLoop(vision.getX(),xBoiler,1,xMax);
				drive.mecanumDrive_Cartesian(0, 0, x, 0);
			}else{
				drive.mecanumDrive_Cartesian(0, 0, 0, 0);
				autoTimer.reset();
				autoState = 201;
			}
			break;
		case(201):
			//ensure stability of horizontal centering
			drive.mecanumDrive_Cartesian(0, 0, 0, 0);
			if(autoTimer.get() > 0.1){
				if(linedUpToBoilerX()){
					autoState = 202;
				}else{
					//try again
					autoState = 200;
				}
			}
			break;
		case(202):
			//go to ideal distance
			if(!linedUpToBoilerY()){
				double y = pLoop(vision.getY(),yBoiler,1,yMax);
				drive.mecanumDrive_Cartesian(0, y, 0, 0);
			}else{
				drive.mecanumDrive_Cartesian(0, 0, 0, 0);
				autoTimer.reset();
				autoState = 203;
			}
			break;
		case(203):
			//ensure stability of distance
			drive.mecanumDrive_Cartesian(0, 0, 0, 0);
			if(autoTimer.get() > 0.1){
				if(linedUpToBoilerX()){
					autoState = 0;
				}else{
					//try again
					autoState = 202;
				}
			}
			break;
		}
	}
	
	/**
	 * 	Simple PID controller, with only a P term
	 * @param sensor current detected tape in pixels
	 * @param target ideal position for tape in pixels
	 * @param maxSpeed how quickly to correct for error. high values could be unstable
	 * @param maxSensor maximum sensor value (xMax or yMax), used to normalize
	 * @return normalized output to give to motor controller
	 */
	private double pLoop(double sensor, double target, double maxSpeed, double maxSensor){
		double error = target - sensor;
		double normalizedError = error / maxSensor;
		return normalizedError * maxSpeed;
	}

	/**
	 * @param value this is the input
	 * @return val-tol < tar < val+tol
	 */
	private boolean numberInTolerance(double value, double target, double tolerance){
		return (value - tolerance < target) && (target < value + tolerance);
	}
	
}
