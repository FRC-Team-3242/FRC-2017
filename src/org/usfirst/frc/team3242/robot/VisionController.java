package org.usfirst.frc.team3242.robot;

import com.ctre.PigeonImu;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;

public class VisionController {
	
	private VisionServer vision;
	private RobotDrive drive;
	private PIDController angleController;
	
	//both tolerances within 0.5%
	private final double xTolerance = 6.4;
	private final double yTolerance = 3.6;
	private final double gyroTolerance = 1.8;
	
	private final double xMax = 640;
	private final double yMax = 480;
	
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
	
	//pid controllers
	private PIDController xAngle;
	private PIDController xStrafe;
	private PIDController yDrive;
	
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
	PigeonImu imu;
	
	public VisionController(VisionServer vision, RobotDrive drive, PIDController angleController, PigeonImu imu){
		this.vision = vision;
		this.drive = drive;
		autoState = 0;
		closestIdealAngle = 0;
		autoTimer = new Timer();
		autoTimer.start();
		this.angleController = angleController;
		this.imu = imu;
		VisionSourceX visionSourceX = new VisionSourceX();
		xAngle = new PIDController(0.8,0.01,0,visionSourceX,(a) -> {drive.mecanumDrive_Cartesian(0, 0, a, 0);});
		xAngle.setInputRange(0, xMax);
		xAngle.setPercentTolerance(0.833);
		xAngle.setSetpoint(xBoiler);
		xStrafe = new PIDController(0.8,0.01,0,visionSourceX,(a) -> {drive.mecanumDrive_Cartesian(a, 0, 0, 0);});
		xStrafe.setInputRange(0, xMax);
		xStrafe.setPercentTolerance(0.833);
		xStrafe.setSetpoint(xLift);
		yDrive = new PIDController(0.8,0.01,0,new VisionSourceY(),(a) -> {drive.mecanumDrive_Cartesian(0, a, 0, 0);});
		yDrive.setInputRange(0, yMax);
		yDrive.setPercentTolerance(0.833);
	}
	
	public double getAbsoluteIMUAngle(){
		double[] ypr = new double[3];
		imu.GetYawPitchRoll(ypr);
		double direction = Math.signum(ypr[0]);
		double yaw = Math.abs(ypr[0]) % 360.0;// -90 + 360
		if(direction < 0){
			yaw = 360.0 - yaw;
		}
		return yaw;
	}
	
	
	public void startLiftTracking(){
		autoState = 100;
	}
	
	public void startBoilerTracking(){
		autoState = 200;
	}
	
	public void stopAll(){
		autoState = 0;
		xAngle.disable();
		xStrafe.disable();
		yDrive.disable();
		angleController.disable();
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
		return numberInTolerance(getAbsoluteIMUAngle(), closestIdealAngle, gyroTolerance);
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
			currentAngle = getAbsoluteIMUAngle();
			if (currentAngle < 30 && currentAngle > 0 || currentAngle < 0 && currentAngle > 330){
				angleController.setSetpoint(angleA);
			}
			else if (currentAngle < 330 && currentAngle > 270){
				angleController.setSetpoint(angleB);
			}
			else if (currentAngle < 90 && currentAngle > 30){
				angleController.setSetpoint(angleC);
			}
			angleController.enable();
			autoState = 101;
			break;
		case(101):
			//adjust to nearest angle
			if (angleController.onTarget()){
				angleController.disable();
				drive.mecanumDrive_Cartesian(0, 0, 0, 0);
				xStrafe.enable();
				autoState = 102;
			}
			break;
		case(102):
			//adjust to optimal x (horizontal positioning) value for lift
			if(xStrafe.onTarget()){
				xStrafe.disable();
				drive.mecanumDrive_Cartesian(0, 0, 0, 0);
				yDrive.setSetpoint(yLift);
				yDrive.enable();
				autoState = 103;
			}
			
			break;
		case(103):
			//adjust to optimal y (distance) value for lift
			if(yDrive.onTarget()){
				//done
				yDrive.disable();
				drive.mecanumDrive_Cartesian(0, 0, 0, 0);
				autoState = 0;
			}
			break;
			
		case(200):
			xAngle.enable();
			autoState = 201;
			break;
			
		case(201):
			//start boiler sequence, center horizontally
			if(xAngle.onTarget()){
				xAngle.disable();
				drive.mecanumDrive_Cartesian(0, 0, 0, 0);
				yDrive.setSetpoint(xBoiler);
				yDrive.enable();
				autoState = 202;
			}
			break;
		case(202):
			//go to ideal distance
			if(yDrive.onTarget()){
				//done
				yDrive.disable();
				drive.mecanumDrive_Cartesian(0, 0, 0, 0);
				autoState = 0;
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

	/**
	 * @param value this is the input
	 * @return val-tol < tar < val+tol
	 */
	private boolean numberInTolerance(double value, double target, double tolerance){
		return (value - tolerance < target) && (target < value + tolerance);
	}
	
	class VisionSourceX implements PIDSource{
		public VisionSourceX(){
			//nothing to instantiate
		}

		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {
			//do nothing, always use displacement
		}

		@Override
		public PIDSourceType getPIDSourceType() {
			return PIDSourceType.kDisplacement;
		}

		@Override
		public double pidGet() {
			return vision.getX();
		}
	}
	
	class VisionSourceY implements PIDSource{
		public VisionSourceY(){
			//nothing to instantiate
		}

		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {
			//do nothing, always use displacement
		}

		@Override
		public PIDSourceType getPIDSourceType() {
			return PIDSourceType.kDisplacement;
		}

		@Override
		public double pidGet() {
			return vision.getY();
		}
	}
}