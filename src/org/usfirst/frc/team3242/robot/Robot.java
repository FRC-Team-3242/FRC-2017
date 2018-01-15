package org.usfirst.frc.team3242.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	final String frontGearAuto = "Front Gear";
	final String dumbFrontGearAuto = "Distance Front Gear";
	final String leftGearAuto = "Left Gear";
	final String rightGearAuto = "Right Gear";
	final String shootingAuto = "Shooting";
	final String nothingAuto = "Do Nothing";
	final String forwardAuto = "Move Forward";
	final String shootThenStrafeAuto = "Static shoot then strafe";
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();
	SendableChooser<String> controllerChooser = new SendableChooser<>();

	private Shooter shooter;
	private BallPickup ballPickup;
	private GearDropper gearDropper;
	private Climber climber;
	private Toggle shooterToggle, ballPickupToggle;
	private XboxController primaryController;
	private XboxController secondaryController;
	private VisionServer gearVision, boilerVision;
	private CameraServer displayVision;
	private VisionController visionController;
	private Encoder driveEncoder;
	private MecanumDrive drive;
	private PigeonIMU.GeneralStatus genStatus;
	private PIDController angleController;
	int turnScalar;
	int autoState;
	private Timer autoTimer;
	final private double autoSpeed = 0.5;
	final private double autoTurnSpeed = 0.5;


	@Override
	public void robotInit() {
		this.setupSmartDashBoard();
		this.initializeObjects();

		//sets up direct camera stream through USB
		UsbCamera cam = displayVision.startAutomaticCapture();
		cam.setResolution(320, 240);
		cam.setFPS(15);


	}

	private void setupSmartDashBoard(){
		chooser.addObject("Front Gear", frontGearAuto);
		chooser.addObject("Dumb Front Gear", dumbFrontGearAuto);
		chooser.addObject("Left Gear", leftGearAuto);
		chooser.addObject("Right Gear", rightGearAuto);
		chooser.addObject("Shooting", shootingAuto);
		chooser.addDefault("Do Nothing", nothingAuto);//shootThenStrafeAuto
		chooser.addObject("Move Forward (pass base line)", forwardAuto);
		chooser.addObject("shoot then strafe", shootThenStrafeAuto);
		SmartDashboard.putData("Auto choices", chooser);

		controllerChooser.addDefault("primary", "primary");
		controllerChooser.addObject("secondary", "secondary");
	}

	/**
	 * Initializes objects and does some setup for them
	 */
	private void initializeObjects(){
		//only using one channel, so distance per pulse is / by 2
		driveEncoder = new Encoder(8, 9, false, CounterBase.EncodingType.k2X);
		driveEncoder.setDistancePerPulse(18.0/ (1440.0 / 2.0));

		drive = new MecanumDrive(new WPI_TalonSRX(8), new WPI_TalonSRX(5), new WPI_TalonSRX(7), new WPI_TalonSRX(6));

		PigeonIMU imu = new PigeonIMU(9);

		angleController = new PIDController(0.8, 0.01,0,new PIDHeadingInput(imu),
				(num) -> {drive.driveCartesian(0, 0, num, 0);});
		angleController.setInputRange(0, 360);
		angleController.setPercentTolerance(1);
		angleController.setContinuous();

		primaryController = new XboxController(1);
		secondaryController = new XboxController(2);

		Encoder shooterEncoder = new Encoder(6, 7, false, CounterBase.EncodingType.k2X);
		shooterEncoder.setDistancePerPulse(0.025);//1/40 (40 pulses per rotation)
		shooterEncoder.setPIDSourceType(PIDSourceType.kRate);

		shooter = new Shooter(new WPI_TalonSRX(3), shooterEncoder, new Spark(2));
		shooter.setSpeedTolerance(20);
		shooter.setRPM(2040); //make adjustable by smartdashboard?

		ballPickup = new BallPickup(new WPI_TalonSRX(1), new WPI_TalonSRX(2));
		gearDropper = new GearDropper(new Spark(4), new AnalogInput(0));
		climber = new Climber(new WPI_TalonSRX(4));

		gearVision = new VisionServer("A");
		boilerVision = new VisionServer("B");
		visionController = new VisionController(gearVision, boilerVision, drive,
				imu, new Relay(3));

		displayVision = CameraServer.getInstance();

		shooterToggle = new Toggle();
		ballPickupToggle = new Toggle();
		autoTimer = new Timer();
		autoTimer.start();
	}
	public void sendInfoToDashboard(){
		SmartDashboard.putBoolean("Gear in sight", gearVision.targetFound());
		SmartDashboard.putBoolean("Boiler in sight", boilerVision.targetFound());
		SmartDashboard.putNumber("shooter RPM", shooter.getRPM());
		SmartDashboard.putNumber("shooter RPS", shooter.getRPS());
		SmartDashboard.putNumber("heading", visionController.getAbsoluteIMUAngle());
		SmartDashboard.putNumber("drive dist", driveEncoder.getDistance());
		SmartDashboard.putNumber("gear dropper potentiometer average", gearDropper.getAvgPotentiometerVal());
		SmartDashboard.putNumber("gear dropper potentiometer", gearDropper.getPotentiometerVal());
		SmartDashboard.putBoolean("isenabled", shooter.isEnabled());
	}


	@Override
	public void teleopInit(){
		visionController.stopAll();
		visionController.search();
		shooter.disable();
	}

	@Override
	public void teleopPeriodic() {
		//if(controllerChooser.getSelected() == "primary"){
			primaryControl();
		//}else{
			//secondaryControl();
		//}
		visionController.update();
		sendInfoToDashboard();
	}

	/**
	 * TODO
	 * LTrigger:		manually control shooter elevator
	 * RTrigger:		manually control shooter
	 *
	 * LBumper:			boiler vision tracking
	 * RBumper:			lift vision tracking
	 * start button:	search for targets
	 * X:				stop all vision tracking
	 *
	 * A:				toggle shooter
	 * B:				run ball pick up
	 * Y:				drop gear
	 */
	public void primaryControl(){
		if(visionController.getAutoState() == 0){
			if(primaryController.getBumper(Hand.kLeft)){
				//visionController.startBoilerTracking();
			}
			if(primaryController.getBumper(Hand.kRight)){
				//visionController.startLiftTracking();
			}
			if(primaryController.getStartButton()){
				//visionController.search();
			}

			double x = primaryController.getRawAxis(4);
			double y = primaryController.getRawAxis(1) * 0.85;
			double r = primaryController.getRawAxis(0) * 0.5;
			if(Math.abs(x) < 0.1){
				x = 0;
			}
			if(Math.abs(y) < 0.1){
				y = 0;
			}
			if(Math.abs(r) < 0.1){
				r = 0;
			}
			drive.driveCartesian(x, y, r, 0);
		}


		if(primaryController.getXButton()){
			//visionController.stopAll();
		}

		shooterToggle.toggle(primaryController.getAButton());
		if (primaryController.getAButton() && !shooter.isEnabled()){
			shooter.enable();
		}else if(!primaryController.getAButton() && shooter.isEnabled()){
			shooter.disable();
		}

		shooter.elevate();

		climber.climb(primaryController.getPOV() == 0, primaryController.getPOV() == 180);//up, down

		gearDropper.open(primaryController.getYButton());


		ballPickupToggle.toggle(primaryController.getBButton());
		ballPickup.set(ballPickupToggle.getStatus(), primaryController.getXButton());
	}

	public void secondaryControl(){

		double RT = secondaryController.getRawAxis(3);
		double LT = secondaryController.getRawAxis(2);
		double LY = secondaryController.getRawAxis(1);
		double RY = secondaryController.getRawAxis(5);

		//ANGLE FINDER


		//SHOOTER []
		if(Math.abs(RT) > 0.1)
			shooter.overrideShooter(RT);		//RT
		else
			shooter.overrideShooter(0);

		//ELEVATOR SHOOTER
		if(Math.abs(LT) > 0.1)
			shooter.overrideElevator(-LT);	//LT
		else
			shooter.overrideElevator(0);


		if(secondaryController.getRawButton(5)){					//L1
			//GEAR DROPPER AUTO
			gearDropper.open(secondaryController.getRawButton(6));	//R1
		}else{
			//GEAR DROPPER MANUAL
			gearDropper.set(secondaryController.getAButton(), secondaryController.getYButton());
		}

		//BALL PICKUP
		if(Math.abs(RY) > 0.1)
			ballPickup.set(RY);				//RY (two motors)
		else
			ballPickup.set(0);

		//CLIMBER
		//climber.climb(secondaryController.getYButton(), secondaryController.getAButton());
	}

	@Override
	public void autonomousInit() {
		autoSelected = chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		autoTimer.reset();
		System.out.println("Auto selected: " + autoSelected);
		if(autoSelected.equals(shootingAuto) &&
				DriverStation.getInstance().getAlliance() == Alliance.Blue){
			turnScalar = -1;
		}else{
			turnScalar = 1;
		}
		autoState = 0;
	}

	@Override
	public void autonomousPeriodic() {
		double currentAngle = visionController.getAbsoluteIMUAngle();


		switch (autoSelected) {

		case nothingAuto:
			break;

		case forwardAuto:
			if(driveEncoder.getDistance() < 84){//84 when fix dist per pulse
				drive.driveCartesian(0, -autoSpeed, 0, 0);
			}
			else{
				drive.driveCartesian(0, 0, 0, 0);
			}
			break;

		case shootThenStrafeAuto:
			switch(autoState){
			case 0:
				ballPickup.set(true, false);
				shooter.enable();
				shooter.elevate();
				if(autoTimer.get() > 6){//TODO
					autoTimer.reset();
					shooter.disable();
					autoState++;
				}
				break;
			case 1:
				ballPickup.set(0);
				shooter.disable();
				shooter.elevate();
				drive.driveCartesian(-0.65, 0, 0, 0);
				if(autoTimer.get() > 4){//TODO
					drive.driveCartesian(0, 0, 0, 0);
					autoState++;
				}
				break;
			case 2:
				drive.driveCartesian(0, 0, 0, 0);
				break;
			}

			break;

		case shootingAuto:

			//flips heading to counter-clockwise when starting from other side of field
			if (DriverStation.getInstance().getAlliance() == Alliance.Blue){
				currentAngle = (360 - currentAngle) % 360;
			}

			switch (autoState){

			case 0:
				if (driveEncoder.getDistance() < 12){
					drive.driveCartesian(0, -autoSpeed, 0, 0);
				}
				else{
					autoState++;
				}
				break;

			case 1:
				if (currentAngle  < 45){
					drive.driveCartesian(0, 0, -autoTurnSpeed * turnScalar, 0);
				}
				else{
					autoState++;
					driveEncoder.reset();
				}
				break;
			case 2:
				if (driveEncoder.getDistance() < 53.5){
					drive.driveCartesian(0, -autoSpeed, 0, 0);
				}
				else{
					autoState++;
				}
				break;
			case 3:
				if (currentAngle < 135){
					drive.driveCartesian(0, 0, -autoTurnSpeed * turnScalar, 0);
				}
				else{
					autoState++;
					driveEncoder.reset();
				}
				break;
			case 4:
				if (driveEncoder.getDistance() < 30){
					drive.driveCartesian(0, -autoSpeed, 0, 0);
				}
				else{
					autoState++;
				}
				break;
			case 5:
				visionController.startBoilerTracking();
				autoState++;
				break;
			case 6:
				if(visionController.linedUpToBoiler()){
					autoState++;
				}
				visionController.update();
				break;
			case 7:
				if(visionController.linedUpToBoiler()){
					if (!shooter.isEnabled()){
						shooter.enable();
					}
				}
				else{
					autoState--;
				}
				break;

			}
			break;

		case rightGearAuto:
			switch (autoState){
			case 0:
				//go forward 12 inches
				if (driveEncoder.getDistance() < 12){
					drive.driveCartesian(0, -autoSpeed, 0, 0); // go forward at 75% speed
				}
				else{
					autoState++;
				}
				break;
			case 1:
				if (currentAngle <= 30){ // turn 30 degrees
					drive.driveCartesian(0, 0, -autoTurnSpeed * turnScalar, 0); // rotate right at 75% speed
				}
				else{
					autoState++;
					driveEncoder.reset();
				}
				break;
			case 2:
				if (driveEncoder.getDistance() < 66.25){ // go forward 66.25 inches
					drive.driveCartesian(0, -autoSpeed, 0, 0); // go forward at 75% speed
				}
				else{
					autoState++;
				}
				break;
			case 3:
				if (currentAngle < 295 || currentAngle > 305){ // rotate to around 300 degrees
					drive.driveCartesian(0, 0, autoTurnSpeed * turnScalar, 0); // rotate left at 75% speed
				}
				else{
					autoState++;
					driveEncoder.reset();
				}
				break;
			case 4:
				if (driveEncoder.getDistance() < 42){
					drive.driveCartesian(0, -autoSpeed, 0, 0); // go forward at 75% speed
				}
				else{
					autoState++;
				}
				break;
			case 5:
				visionController.startLiftTracking();
				autoState++;
				break;
			case 6:
				visionController.update();
				// to_do: use auto gear placing function
				break;
			}
			break;
		case leftGearAuto:
			//go forward 78.5 inches
		switch (autoState){
			case 0:
				if (driveEncoder.getDistance() < 78.5){
					drive.driveCartesian(0, -autoSpeed, 0, 0); // go forward at 75% speed
				}
				else{
					autoState++;
				}
				break;
			case 1:
				if (currentAngle <= 60){ // turn 60 degrees
					drive.driveCartesian(0, 0, -autoTurnSpeed * turnScalar, 0); // rotate at 75% speed
				}
				else{
					autoState++;
					driveEncoder.reset();
				}
				break;
			case 2:
				if (driveEncoder.getDistance() < 25){
					drive.driveCartesian(0, -autoSpeed, 0, 0); // go forward at 75% speed
				}
				else{
					autoState++;
				}
				break;
			case 3:
				visionController.startLiftTracking();
				autoState++;
				break;
			case 4:
				visionController.update();
				break;
		}
		break;
		case dumbFrontGearAuto:
			switch (autoState){

			case 0:
				// 1. Move forward
				if (driveEncoder.getDistance() < 51){ // The distance to go forward
					drive.driveCartesian(0, -autoSpeed, 0, 0); // go forward at 75% speed
				}else{
					autoTimer.reset();
					autoState++;
				}
				break;
			case 1:
				gearDropper.open(true);
				if(autoTimer.get() < 0.3){
					autoTimer.reset();
					autoState++;
				}
				break;
			case 2:
				gearDropper.open(true);
				drive.driveCartesian(0, 0.1, 0, 0);//back up slowly
				if(autoTimer.get() > 1){
					drive.driveCartesian(0, 0, 0, 0);
					autoState++;
				}
				break;
			case 3:
				drive.driveCartesian(0, 0, 0, 0);
				gearDropper.open(false);//close gear again
				break;
			}
			break;
		case frontGearAuto:
				switch (autoState){

				case 0:
					// 1. Move forward
					if (driveEncoder.getDistance() < 68){ // The distance to go forward
						drive.driveCartesian(0, -autoSpeed, 0, 0); // go forward at 75% speed
					}
					else{
						autoState++;
					}
					break;
				case 1:
						visionController.startLiftTracking();
						autoState++;
						break;
				case 2:
					visionController.update();
					// to_do: 2. Use auto gear placing function
					//
					break;
				}
				break;
		}
	}

	@Override
	public void testPeriodic() {

	}

}

