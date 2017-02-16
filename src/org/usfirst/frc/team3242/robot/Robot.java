package org.usfirst.frc.team3242.robot;

import com.ctre.CANTalon;
import com.ctre.PigeonImu;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDController.Tolerance;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.	
 */
public class Robot extends IterativeRobot {
	final String frontGearAuto = "Front Gear";
	final String leftGearAuto = "Left Gear";
	final String rightGearAuto = "Right Gear";
	final String shootingAuto = "Shooting";
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();

	Shooter shooter;
	BallPickup ballPickup;
	Joystick controller;
	VisionServer vision;
	VisionController visionController;
	Encoder driveEncoder;
	RobotDrive drive;
	PigeonImu imu;
	PigeonImu.GeneralStatus genStatus;
	PIDController angleController;
	boolean turnOne;
	boolean turnTwo;
	boolean readyToTurn;
	boolean startedTracking;
	int turnScalar;

	
	double[] ypr;

	
	@Override
	public void robotInit() {
		chooser.addDefault("Front Gear", frontGearAuto);
		chooser.addObject("Left Gear", leftGearAuto);
		chooser.addObject("Right Gear", rightGearAuto);
		chooser.addObject("Shooting", shootingAuto);
		SmartDashboard.putData("Auto choices", chooser);
		
		driveEncoder = new Encoder(0, 1, false, CounterBase.EncodingType.k4X); 
		driveEncoder.setDistancePerPulse(0.014);
		
		drive = new RobotDrive(new CANTalon(0), new CANTalon(1), new CANTalon(2), new CANTalon(3));
		
		imu = new PigeonImu(20);
		ypr =  new double[3];
		
		angleController = new PIDController(0.8, 0.01,0,new PIDHeadingInput(imu),
				(num) -> {drive.mecanumDrive_Cartesian(0, 0, num, 0);});
		angleController.setInputRange(0, 360);
		angleController.setOutputRange(-100, 100);
		angleController.setPercentTolerance(5);
		angleController.setContinuous();
		
		controller = new Joystick(1);
		shooter = new Shooter(new CANTalon(4), new Encoder(0, 1, false, CounterBase.EncodingType.k4X), new CANTalon(5));
		shooter.setSpeedTolerance(20);
		ballPickup = new BallPickup(new CANTalon(6), new CANTalon(7));
		
		vision = new VisionServer();
		visionController = new VisionController(vision, drive, angleController, imu);
	}
	
	@Override
	public void autonomousInit() {
		autoSelected = chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
		//with current setup, the turnScalar will mess up the inequalities
		if(DriverStation.getInstance().getAlliance() == Alliance.Blue){
			turnScalar = -1;
		}else{
			turnScalar = 1;
		}
		turnOne = false;
		turnTwo = false;
		readyToTurn = false;
		startedTracking = false;
	}

	@Override
	public void autonomousPeriodic() {
		double currentAngle = visionController.getAbsoluteIMUAngle();
		
		//flips heading to counter-clockwise when starting from other side of field
		if (DriverStation.getInstance().getAlliance() == Alliance.Blue){
			currentAngle = (360 - currentAngle) % 360;
		}
		
		switch (autoSelected) {
		
		case shootingAuto:
			if (driveEncoder.getDistance() < 12 && !turnOne){
				drive.mecanumDrive_Cartesian(0, 0.75, 0, 0);
				break;
			}
			
			if (currentAngle  < 45 && !turnOne){
				drive.mecanumDrive_Cartesian(0, 0, 0.75 * turnScalar, 0);
				break;
			}
			if (!turnOne){
				turnOne = true;
				driveEncoder.reset();
			}
			if (driveEncoder.getDistance() < 53.5 && turnOne){
				drive.mecanumDrive_Cartesian(0, 0.75, 0, 0);
				break;
			}
			if (currentAngle < 135 && !turnTwo){
				drive.mecanumDrive_Cartesian(0, 0, 0.75 * turnScalar, 0);
				break;
			}
			if (!turnTwo){
				turnTwo = true;
				driveEncoder.reset();
			}
			if (driveEncoder.getDistance() < 30){
				drive.mecanumDrive_Cartesian(0, 0.75, 0, 0);
				break;
			}
			
			if( driveEncoder.getDistance() > 30 && turnTwo && !readyToTurn){
				readyToTurn = true;
			}
			if(readyToTurn && !startedTracking){
				visionController.startBoilerTracking();
				startedTracking = true;
			}
			visionController.update();
			
			
			
			
		case rightGearAuto:
			//go forward 12 inches
			if (driveEncoder.getDistance() < 12 && !turnOne){ 
				drive.mecanumDrive_Cartesian(0, 0.75, 0, 0); // go forward at 75% speed
				break;
			}
			
			if (currentAngle <= 30 && !turnOne){ // turn 30 degrees
				drive.mecanumDrive_Cartesian(0, 0, 0.75 * turnScalar, 0); // rotate right at 75% speed
				break;
			}
			if (!turnOne){
				turnOne = true;
				driveEncoder.reset();
			}

			if (driveEncoder.getDistance() < 66.25 && turnOne){ // go forward 66.25 inches
				drive.mecanumDrive_Cartesian(0, 0.75, 0, 0); // go forward at 75% speed
				break;
			}
			
			if (currentAngle < 295 || currentAngle > 305 && !turnTwo){ // rotate to around 300 degrees
				drive.mecanumDrive_Cartesian(0, 0, -0.75 * turnScalar, 0); // rotate left at 75% speed
				break;
			}
			
			if (!turnTwo){
				turnTwo = true;
				driveEncoder.reset();
				
			}
			
			if (driveEncoder.getDistance() < 42 && turnTwo){
				drive.mecanumDrive_Cartesian(0, 0.75, 0, 0); // go forward at 75% speed
				break;
			}
			if( driveEncoder.getDistance() > 42 && turnTwo && !readyToTurn){
				readyToTurn = true;
			}
			if(readyToTurn && !startedTracking){
				visionController.startLiftTracking();
				startedTracking = true;
			}
			visionController.update();
			// to_do: use auto gear placing function
			break;
		case leftGearAuto:
			//go forward 78.5 inches
			if (driveEncoder.getDistance() < 78.5 && !turnOne){ 
				drive.mecanumDrive_Cartesian(0, 0.75, 0, 0); // go forward at 75% speed
				break;
			}
			if (currentAngle <= 60){ // turn 60 degrees
				drive.mecanumDrive_Cartesian(0, 0, 0.75 * turnScalar, 0); // rotate at 75% speed
				break;
			}
			if (!turnOne){
				turnOne = true;
				driveEncoder.reset();
				
			}

			if (driveEncoder.getDistance() < 25 && turnOne){
				drive.mecanumDrive_Cartesian(0, 0.75, 0, 0); // go forward at 75% speed
				break;
			}
			
			if (driveEncoder.getDistance() > 25 && !readyToTurn){
				readyToTurn = true;
			}
			if(readyToTurn && !startedTracking){
				visionController.startLiftTracking();
				startedTracking = true;
			}
			
			visionController.update();
			
			// to_do: use auto gear placing function (done?)
			
			break;
			
		case frontGearAuto:
			default:
				// 1. Move forward
				if (driveEncoder.getDistance() < 68){ // The distance to go forward
					drive.mecanumDrive_Cartesian(0, 0.75, 0, 0); // go forward at 75% speed
					break;
				}
				
				if (driveEncoder.getDistance() > 68 && !readyToTurn){
					readyToTurn = true;
				}
				if(readyToTurn && !startedTracking){
					visionController.startLiftTracking();
					startedTracking = true;
				}
				
				visionController.update();
				// to_do: 2. Use auto gear placing function
				//
				break;
			
		}
	}

	@Override
	public void teleopPeriodic() {
		drive.mecanumDrive_Cartesian(controller.getRawAxis(4), controller.getRawAxis(1), controller.getRawAxis(0), 0);
		
		if (controller.getRawButton(4) && !shooter.isEnabled()){
			shooter.enable();
			shooter.setRPM(5000); //make adjustable by smartdashboard?
			
		}
		else if (controller.getRawButton(4) && shooter.isEnabled()){
			shooter.disable();
		}
		shooter.elevate();
		
		ballPickup.set(controller.getRawButton(2));
	}

	
	@Override
	public void testPeriodic() {
		
	}
}

