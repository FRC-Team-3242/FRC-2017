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
	private final double angleB = 300;
	private final double angleC = 60;
	
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
	int turningDirection;
	double currentAngle;
	
	
	public VisionController(VisionServer vision, RobotDrive drive, PigeonImu gyro){
		this.vision = vision;
		this.drive = drive;
		autoState = 0;
		closestIdealAngle = 0;
		autoTimer = new Timer();
		autoTimer.start();
		this.gyro = gyro;
		gyro.SetFusedHeading(360 * 32);
	}
	
	public double getNormalIMUAngle(){
		gyro.GetGeneralStatus(new PigeonImu.GeneralStatus());
		double currentAngle = gyro.GetFusedHeading(new PigeonImu.FusionStatus());
		return Math.abs(currentAngle) % 360;
	}
	
	public static double angleConverter(double angle){
		return angle % 360;
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
	public boolean correctPegAngle(){
		return numberInTolerance(getNormalIMUAngle(), closestIdealAngle, gyroTolerance);
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
			//select nearest ideal angle
			
			currentAngle = getNormalIMUAngle();
			if (currentAngle < 30 && currentAngle > 0 || currentAngle < 0 && currentAngle > 330){
				closestIdealAngle = angleA;
			}
			else if (currentAngle < 330 && currentAngle > 270){
				closestIdealAngle = angleB;
			}
			else if (currentAngle < 90 && currentAngle > 30){
				closestIdealAngle = angleC;
			}
			autoState = 101;
			break;
		case(101):
			//adjust to nearest angle
			currentAngle = getNormalIMUAngle();
			if (!correctPegAngle()){
				drive.mecanumDrive_Cartesian(0, 0, turningDirection * pLoopAngle(currentAngle, closestIdealAngle, 1, 60), 0);
			}
			else {
				drive.mecanumDrive_Cartesian(0, 0, 0, 0);
				autoTimer.reset();
				autoState = 102;
			}
			break;
		case(102):
			//ensure stability of peg angle positioning
			drive.mecanumDrive_Cartesian(0, 0, 0, 0);
			if(autoTimer.get() > 0.1){
				if(correctPegAngle()){
					autoState = 103;
				}else{
					//try again
					autoState = 101;
				}

			}
			break;
			
		case(103):
			//adjust to optimal x (horizontal positioning) value for lift
			if(!linedUpToLiftX()){
				double x = pLoop(vision.getX(),xLift,1,xMax);
				drive.mecanumDrive_Cartesian(x, 0, 0, 0);
			}else{
				drive.mecanumDrive_Cartesian(0, 0, 0, 0);
				autoTimer.reset();
				autoState = 104;
			}
			
			break;
		case(104):
			//ensure stability of x (horizontal positioning) value for lift
			drive.mecanumDrive_Cartesian(0, 0, 0, 0);
			if(autoTimer.get() > 0.1){
				if(linedUpToLiftX()){
					autoState = 105;
				}else{
					//try again
					autoState = 103;
				}
			}
			break;
		case(105):
			//adjust to optimal y (distance) value for lift
			if(!linedUpToLiftY()){
				double y = pLoop(vision.getY(),yLift,1,yMax);
				drive.mecanumDrive_Cartesian(0, y, 0, 0);
			}else{
				drive.mecanumDrive_Cartesian(0, 0, 0, 0);
				autoTimer.reset();
				autoState = 106;
			}
			break;
			
		case(106):
			//ensure stability of y (distance) value for lift
			drive.mecanumDrive_Cartesian(0, 0, 0, 0);
			if(autoTimer.get() > 0.1){
				if(linedUpToLiftX()){
					autoState = 0;
				}else{
					//try again
					autoState = 105;
				}
			}
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
	 * @param maxSpeed how quickly to correct for error. high values could be unstable. between 0 and 1.
	 * @param maxSensor maximum sensor value (xMax or yMax), used to normalize
	 * @return normalized output to give to motor controller
	 */
	private double pLoop(double sensor, double target, double maxSpeed, double maxSensor){
		double error = target - sensor;
		double normalizedError = error / maxSensor;
		return normalizedError * maxSpeed;
	}
	
	public double pLoopAngle(double sensor, double target, double maxSpeed, double maxSensor){
		
		double error = 360 - sensor + target;
		
		if (error > 180){
			error = sensor - target;
		}
		
		if (Math.abs(closestIdealAngle - currentAngle)<180){
			turningDirection = -1;
		}
		else if (Math.abs(closestIdealAngle - currentAngle)>=180){
			turningDirection = 1;
		}
		
		if (turningDirection == -1 && closestIdealAngle > currentAngle){
			turningDirection *= -1;
		}
		else if (turningDirection == 1 && closestIdealAngle < currentAngle){
			turningDirection *= -1;
		}
		
		double normalizedError = error / maxSensor;
		return normalizedError * maxSpeed * turningDirection;
	}
	

	/**
	 * @param value this is the input
	 * @return val-tol < tar < val+tol
	 */
	private boolean numberInTolerance(double value, double target, double tolerance){
		return (value - tolerance < target) && (target < value + tolerance);
	}
	
}
