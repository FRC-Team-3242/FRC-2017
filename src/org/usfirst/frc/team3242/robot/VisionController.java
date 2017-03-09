package org.usfirst.frc.team3242.robot;

import com.ctre.PigeonImu;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;

public class VisionController {
	
	private RobotDrive drive;
	private PIDController angleController;
	
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
	
	private double x;
	private double y;
	private double r;
	
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
			RobotDrive drive, PigeonImu imu, Relay lights){
		this.drive = drive;
		autoState = 0;
		autoTimer = new Timer();
		autoTimer.start();
		this.imu = imu;
		this.boilerVision = boilerVision;
		this.gearVision = gearVision;
		lights.setDirection(Direction.kBoth);
		//lights.setDirection(Relay.Direction.kForward);
		
		xBoilerController = new PIDController(0.8,0.01,0,new VisionSourceX(boilerVision),
				(a) -> {r = a;});
		xBoilerController.setInputRange(0, xMax);
		xBoilerController.setPercentTolerance(0.833);
		xBoilerController.setSetpoint(xBoilerIdeal);
		yBoilerController = new PIDController(0.8,0.01,0,new VisionSourceY(boilerVision),
				(a) -> {y = a;});
		yBoilerController.setInputRange(0, yMax);
		yBoilerController.setPercentTolerance(0.833);
		yBoilerController.setSetpoint(yBoilerIdeal);
		
		xGearController = new PIDController(0.8,0.01,0,new VisionSourceX(gearVision),
				(a) -> {x = a;});
		xGearController.setInputRange(0, xMax);
		xGearController.setPercentTolerance(0.833);
		xGearController.setSetpoint(xLiftIdeal);
		yGearController = new PIDController(0.8,0.01,0,new VisionSourceY(gearVision),
				(a) -> {y = a;});
		yGearController.setInputRange(0, yMax);
		yGearController.setPercentTolerance(0.833);
		yGearController.setSetpoint(yLiftIdeal);
		
		angleController = new PIDController(0.8, 0.01,0,new PIDHeadingInput(imu),
				(a) -> {r=a;});
		angleController.setInputRange(0, 360);
		angleController.setPercentTolerance(1);
		angleController.setContinuous();
		
		x = 0;
		y = 0;
		r = 0;
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
		gearVision.enable();
		boilerVision.disable();
		autoState = 100;
	}
	
	public void startBoilerTracking(){
		boilerVision.enable();
		gearVision.disable();
		autoState = 200;
	}
	
	public void search(){
		gearVision.enable();
		boilerVision.enable();
	}
	
	public void stopAll(){
		gearVision.disable();
		boilerVision.disable();
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
	
	public boolean linedUpToLift(){
		return linedUpToLiftX() && linedUpToLiftY();
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
		//this allows multiple PID loops to run simultaneously
		if(autoState != 0){
			drive.mecanumDrive_Cartesian(x, y, r, 0);
		}
		//check if boiler target was lost
		if(autoState >= 100 && autoState < 200){
			if(!boilerVision.targetFound()){
				stopAll();
				//check last known position of target
				if(gearVision.getX() > xMax / 2){
					r = 0.1;
				}else{
					r = -0.1;
				}
				autoState = -100;
			}
		}
		//check if gear target was lost
		if(autoState >= 200 && autoState < 300){
			if(!gearVision.targetFound()){
				stopAll();
				//check last known position of target
				if(gearVision.getX() > xMax / 2){
					x = 0.2;
				}else{
					x = -0.2;
				}
				autoState = -200;
			}
		}
		switch(autoState){
		case(-200):
			//spin to search for gear target
			if(gearVision.targetFound()){
				r = 0;
				startLiftTracking();
			}
			break;
		case(-100):
			//spin to search for boiler target
			if(boilerVision.targetFound()){
				r = 0;
				startBoilerTracking();
			}
			break;
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
				drive.mecanumDrive_Cartesian(0, 0, 0, 0);
				xGearController.enable();
				yGearController.enable();
				autoState = 103;
			}
			break;
		case(102):
			//wait to adjust to optimal x and y value for lift, while maintaining angle
			if(linedUpToLift()){
				drive.mecanumDrive_Cartesian(0, 0, 0, 0);
				stopAll();
			}
			
			break;
			
		case(200):
			//start going to ideal boiler distance and angle
			xBoilerController.enable();
			yBoilerController.enable();
			autoState = 201;
			break;
		case(202):
			//go to ideal distance
			if(linedUpToBoiler()){
				//done
				yBoilerController.disable();
				drive.mecanumDrive_Cartesian(0, 0, 0, 0);
				stopAll();
			}
			break;
		}
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