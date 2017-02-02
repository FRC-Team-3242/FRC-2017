package org.usfirst.frc.team3242.robot;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;

public class VisionController {
	
	private VisionServer vision;
	private RobotDrive drive;
	
	//tolerance within x%
	private double xTolerance = 3;
	private double yTolerance = 2;
	
	private double xMax = 1280;
	private double yMax = 720;
	
	//center point of ideal lift
	private double xLift = 400;
	private double yLift = 600;
	
	//center point of boiler
	private double xBoiler = 700;
	private double yBoiler = 100;
	
	/**
	 * integer for state machine
	 * 0-99		: nothing
	 * 100-199	: lift
	 * 200-299	: boiler
	 */
	private int autoState;
	private Timer autoTimer;
	
	public VisionController(VisionServer vision, RobotDrive drive){
		this.vision = vision;
		this.drive = drive;
		autoState = 0;
		autoTimer = new Timer();
		autoTimer.start();
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
	
	public boolean linedUpToBoiler(){
		return linedUpToBoilerX() && linedUpToBoilerY();
	}
	private boolean linedUpToBoilerX(){
		return numberInTolerance(vision.getX(),xBoiler,xTolerance);
	}
	private boolean linedUpToBoilerY(){
		return numberInTolerance(vision.getY(),yBoiler,yTolerance);
	}
	
	private boolean linedUpToLiftX(){
		return numberInTolerance(vision.getX(),xLift,xTolerance);
	}
	private boolean linedUpToLiftY(){
		return numberInTolerance(vision.getY(),yLift,yTolerance);
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
			//start lift sequence
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
			//ensure distance
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
			//TODO ...
			break;
		}
	}
	
	/**
	 * 	Simple PID controller, with only a P term
	 * @param sensor current detected tape in pixels
	 * @param target ideal position for tape in pixels
	 * @param maxSpeed how quickly to correct for error. high values could be unstable
	 * @param maxPixel maximum pixel value (xMax or yMax), used to normalize
	 * @return normalized output to give to motor controller
	 */
	private double pLoop(double sensor, double target, double maxSpeed, double maxPixel){
		double error = target - sensor;
		double normalizedError = error / maxPixel;
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
