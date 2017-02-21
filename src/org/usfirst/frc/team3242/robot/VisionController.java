package org.usfirst.frc.team3242.robot;

import com.ctre.PigeonImu;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;

public class VisionController {
	
	private RobotDrive drive;
	private PIDController angleController;
	private Relay lights;
	
	//camera resolution
	private final double xMax = 640;
	private final double yMax = 480;
	
	//center point of ideal lift
	private final double xLiftIdeal = 400;
	private final double yLiftIdeal = 600;
	
	//ideal angles for lifts
	private final double angleA = 0;
	private final double angleB = 300;
	private final double angleC = 60;
	
	//center point of boiler
	private final double xBoilerIdeal = 700;
	private final double yBoilerIdeal = 100;
	
	//pid controllers
	private PIDController xBoilerController;
	private PIDController yBoilerController;
	private PIDController xGearController;
	private PIDController yGearController;
	
	/**
	 * integer for state machine
	 * 0-99		: nothing
	 * 100-199	: lift
	 * 200-299	: boiler
	 */
	private int autoState;
	private Timer autoTimer;
	int turningDirection;
	double currentAngle;
	PigeonImu imu;
	VisionServer boilerVision;
	VisionServer gearVision;
	
	public VisionController(VisionServer gearVision, VisionServer boilerVision,
			RobotDrive drive, PIDController angleController, PigeonImu imu, Relay lights){
		this.drive = drive;
		autoState = 0;
		autoTimer = new Timer();
		autoTimer.start();
		this.angleController = angleController;
		this.imu = imu;
		this.boilerVision = boilerVision;
		this.gearVision = gearVision;
		this.lights = lights;
		lights.setDirection(Relay.Direction.kForward);
		
		xBoilerController = new PIDController(0.8,0.01,0,new VisionSourceX(boilerVision),
				(a) -> {drive.mecanumDrive_Cartesian(0, 0, a, 0);});
		xBoilerController.setInputRange(0, xMax);
		xBoilerController.setPercentTolerance(0.833);
		xBoilerController.setSetpoint(xBoilerIdeal);
		yBoilerController = new PIDController(0.8,0.01,0,new VisionSourceY(boilerVision),
				(a) -> {drive.mecanumDrive_Cartesian(0, a, 0, 0);});
		yBoilerController.setInputRange(0, yMax);
		yBoilerController.setPercentTolerance(0.833);
		yBoilerController.setSetpoint(yBoilerIdeal);
		
		xGearController = new PIDController(0.8,0.01,0,new VisionSourceX(gearVision),
				(a) -> {drive.mecanumDrive_Cartesian(a, 0, 0, 0);});
		xGearController.setInputRange(0, xMax);
		xGearController.setPercentTolerance(0.833);
		xGearController.setSetpoint(xLiftIdeal);
		yGearController = new PIDController(0.8,0.01,0,new VisionSourceY(gearVision),
				(a) -> {drive.mecanumDrive_Cartesian(0, a, 0, 0);});
		yGearController.setInputRange(0, yMax);
		yGearController.setPercentTolerance(0.833);
		yGearController.setSetpoint(yLiftIdeal);
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
	
	private void turnOnLights(){
		lights.set(Value.kOn);
	}
	
	public void startLiftTracking(){
		gearVision.enable();
		boilerVision.disable();
		turnOnLights();
		autoState = 100;
	}
	
	public void startBoilerTracking(){
		boilerVision.enable();
		gearVision.disable();
		turnOnLights();
		autoState = 200;
	}
	
	public void search(){
		gearVision.enable();
		boilerVision.enable();
		turnOnLights();
	}
	
	public void stopAll(){
		gearVision.disable();
		boilerVision.disable();
		lights.set(Value.kOff);
		autoState = 0;
		xBoilerController.disable();
		yBoilerController.disable();
		xGearController.disable();
		yGearController.disable();
		angleController.disable();
	}
	
	public int getAutoState(){
		return autoState;
	}
	
	public boolean linedUpToLiftX(){
		return xGearController.onTarget();
	}
	public boolean linedUpToLiftY(){
		return yGearController.onTarget();
	}

	public boolean linedUpToBoiler(){
		return linedUpToBoilerX() && linedUpToBoilerY();
	}
	public boolean linedUpToBoilerX(){
		return xBoilerController.onTarget();
	}
	public boolean linedUpToBoilerY(){
		return yBoilerController.onTarget();
	}
	public boolean correctPegAngle(){
		return angleController.onTarget();
	}
	
	/**
	 * should be called ONCE per iteration at end of iteration
	 * for instance, at the end of the teleopPeriodic function
	 * 
	 * this function executes the controller's state machine
	 */
	public void update(){
		gearVision.update();
		boilerVision.update();
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
				xGearController.enable();
				autoState = 102;
			}
			break;
		case(102):
			//adjust to optimal x (horizontal positioning) value for lift
			if(xGearController.onTarget()){
				xGearController.disable();
				drive.mecanumDrive_Cartesian(0, 0, 0, 0);
				yGearController.enable();
				autoState = 103;
			}
			
			break;
		case(103):
			//adjust to optimal y (distance) value for lift
			if(yGearController.onTarget()){
				//done
				yGearController.disable();
				drive.mecanumDrive_Cartesian(0, 0, 0, 0);
				stopAll();
			}
			break;
			
		case(200):
			xBoilerController.enable();
			autoState = 201;
			break;
			
		case(201):
			//start boiler sequence, center horizontally
			if(xBoilerController.onTarget()){
				xBoilerController.disable();
				drive.mecanumDrive_Cartesian(0, 0, 0, 0);
				yBoilerController.enable();
				autoState = 202;
			}
			break;
		case(202):
			//go to ideal distance
			if(yBoilerController.onTarget()){
				//done
				yBoilerController.disable();
				drive.mecanumDrive_Cartesian(0, 0, 0, 0);
				stopAll();
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
	 * @deprecated use built in PID controller
	 */
	private double pLoop(double sensor, double target, double maxSpeed, double maxSensor){
		double error = target - sensor;
		double normalizedError = error / maxSensor;
		return normalizedError * maxSpeed;
	}

	/**
	 * @param value this is the input
	 * @return val-tol < tar < val+tol
	 * @deprecated use built in PID controller
	 */
	private boolean numberInTolerance(double value, double target, double tolerance){
		return (value - tolerance < target) && (target < value + tolerance);
	}
	
	class VisionSourceX implements PIDSource{
		VisionServer vision;
		public VisionSourceX(VisionServer vision){
			this.vision = vision;
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
		VisionServer vision;
		public VisionSourceY(VisionServer vision){
			this.vision = vision;
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