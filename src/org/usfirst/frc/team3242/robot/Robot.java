package org.usfirst.frc.team3242.robot;

import com.ctre.CANTalon;
import com.ctre.PigeonImu;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.Spark;
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
	final String leftGearAuto = "Left Gear";
	final String rightGearAuto = "Right Gear";
	final String shootingAuto = "Shooting";
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();

	Shooter shooter;
	BallPickup ballPickup;
	GearDropper gearDropper;
	Climber climber;
	Toggle shooterToggle;
	XboxController primaryController;
	XboxController secondaryController;
	VisionServer gearVision, boilerVision;
	VisionController visionController;
	Encoder driveEncoder;
	RobotDrive drive;
	PigeonImu.GeneralStatus genStatus;
	PIDController angleController;
	int turnScalar;
	int autoState;

	
	@Override
	public void robotInit() {
		chooser.addDefault("Front Gear", frontGearAuto);
		chooser.addObject("Left Gear", leftGearAuto);
		chooser.addObject("Right Gear", rightGearAuto);
		chooser.addObject("Shooting", shootingAuto);
		SmartDashboard.putData("Auto choices", chooser);
		
		driveEncoder = new Encoder(6, 7, false, CounterBase.EncodingType.k2X); 
		driveEncoder.setDistancePerPulse(0.014);
		
		drive = new RobotDrive(new CANTalon(8), new CANTalon(5), new CANTalon(7), new CANTalon(6));
		drive.setInvertedMotor(MotorType.kFrontLeft, true);
		drive.setInvertedMotor(MotorType.kRearLeft, true);
		
		PigeonImu imu = new PigeonImu(9);
		
		angleController = new PIDController(0.8, 0.01,0,new PIDHeadingInput(imu),
				(num) -> {drive.mecanumDrive_Cartesian(0, 0, num, 0);});
		angleController.setInputRange(0, 360);
		angleController.setPercentTolerance(1);
		angleController.setContinuous();
		primaryController = new XboxController(1);
		secondaryController = new XboxController(2);
		shooter = new Shooter(new CANTalon(3),
				new Encoder(9, 8, false, CounterBase.EncodingType.k2X), new Spark(0));
		shooter.setSpeedTolerance(20);
		ballPickup = new BallPickup(new CANTalon(1), new CANTalon(2));
		gearDropper = new GearDropper(new Spark(1), new AnalogInput(1));
		climber = new Climber(new CANTalon(4));
		
		gearVision = new VisionServer("A");
		boilerVision = new VisionServer("B");
		visionController = new VisionController(gearVision, boilerVision, drive,
				imu, new Relay(3));
		
		shooterToggle = new Toggle();

		shooter.setRPM(5000); //make adjustable by smartdashboard?
	}
	
	public void sendInfoToDashboard(){
		SmartDashboard.putString("test", "yes");
		SmartDashboard.putBoolean("Gear in sight", gearVision.targetFound());
		SmartDashboard.putBoolean("Boiler in sight", boilerVision.targetFound());
		SmartDashboard.putNumber("shooter RPM", shooter.getRPM());
		SmartDashboard.putNumber("shooter dist", shooter.getdist());
		SmartDashboard.putNumber("heading", visionController.getAbsoluteIMUAngle());
		SmartDashboard.putNumber("drive dist", driveEncoder.getDistance());
		SmartDashboard.putNumber("gear dropper potentiometer average", gearDropper.getAvgPotentiometerVal());
		SmartDashboard.putNumber("gear dropper potentiometer", gearDropper.getPotentiometerVal());
	}
	

	@Override
	public void teleopInit(){
		visionController.stopAll();
		shooter.disable();
	}
	
	@Override
	public void teleopPeriodic() {
		if(primaryController.getPOVCount() == 1){
			//primaryControl();
		}
		if(secondaryController.getPOVCount() == 1){
			secondaryControl();
		}
		visionController.update();
		sendInfoToDashboard();
	}

	/**
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
				visionController.startBoilerTracking();
			}
			if(primaryController.getBumper(Hand.kRight)){
				visionController.startLiftTracking();
			}
			if(primaryController.getStartButton()){
				visionController.search();
			}
			
			double x = primaryController.getRawAxis(0);
			double y = primaryController.getRawAxis(1);
			double r = primaryController.getRawAxis(4);
			if(Math.abs(x) < 0.1){
				x = 0;
			}
			if(Math.abs(y) < 0.1){
				y = 0;
			}
			if(Math.abs(r) < 0.1){
				r = 0;
			}
			drive.mecanumDrive_Cartesian(x, y, r, 0);
		}
		
		shooter.overrideShooter(primaryController.getRawAxis(3));	//RT
		shooter.overrideElevator(-primaryController.getRawAxis(2));	//LT
		
		if(primaryController.getXButton()){
			visionController.stopAll();
		}
		
		if(primaryController.getAButton()){
			shooterToggle.toggle();
		}
		
		if (shooterToggle.getStatus() && !shooter.isEnabled()){
			shooter.enable();
		}
		else if (shooterToggle.getStatus() && shooter.isEnabled()){
			shooter.disable();
		}
		shooter.elevate();
		
		climber.climb(primaryController.getPOV() == 0, primaryController.getPOV() == 180);//up, down
		
		gearDropper.open(primaryController.getYButton());
		
		ballPickup.set(primaryController.getBButton());
	}
	
	public void secondaryControl(){
		shooter.overrideShooter(secondaryController.getRawAxis(3));		//RT
		shooter.overrideElevator(-secondaryController.getRawAxis(2));	//LT
		gearDropper.override(-secondaryController.getRawAxis(1));		//LY
		climber.climb(secondaryController.getYButton(), secondaryController.getAButton());
		ballPickup.set(secondaryController.getRawAxis(5));				//RY
	}
	
	@Override
	public void autonomousInit() {
		autoSelected = chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
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
		
		case shootingAuto:

			//flips heading to counter-clockwise when starting from other side of field
			if (DriverStation.getInstance().getAlliance() == Alliance.Blue){
				currentAngle = (360 - currentAngle) % 360;
			}
			
			switch (autoState){
			
			case 0:
				if (driveEncoder.getDistance() < 12){
					drive.mecanumDrive_Cartesian(0, 0.75, 0, 0);
				}
				else{
					autoState++;
				}
				break;
				
			case 1:
				if (currentAngle  < 45){
					drive.mecanumDrive_Cartesian(0, 0, 0.75 * turnScalar, 0);
				}
				else{
					autoState++;
					driveEncoder.reset();
				}
				break;
			case 2:
				if (driveEncoder.getDistance() < 53.5){
					drive.mecanumDrive_Cartesian(0, 0.75, 0, 0);
				}
				else{
					autoState++;
				}
				break;
			case 3:
				if (currentAngle < 135){
					drive.mecanumDrive_Cartesian(0, 0, 0.75 * turnScalar, 0);
				}
				else{
					autoState++;
					driveEncoder.reset();
				}
				break;
			case 4:
				if (driveEncoder.getDistance() < 30){
					drive.mecanumDrive_Cartesian(0, 0.75, 0, 0);
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
			
			
		case rightGearAuto:
			switch (autoState){
			case 0:
				//go forward 12 inches
				if (driveEncoder.getDistance() < 12){ 
					drive.mecanumDrive_Cartesian(0, 0.75, 0, 0); // go forward at 75% speed
				}
				else{
					autoState++;
				}
				break;
			case 1:
				if (currentAngle <= 30){ // turn 30 degrees
					drive.mecanumDrive_Cartesian(0, 0, 0.75 * turnScalar, 0); // rotate right at 75% speed
				}
				else{
					autoState++;
					driveEncoder.reset();
				}
				break;
			case 2:
				if (driveEncoder.getDistance() < 66.25){ // go forward 66.25 inches
					drive.mecanumDrive_Cartesian(0, 0.75, 0, 0); // go forward at 75% speed
				}
				else{
					autoState++;
				}
				break;
			case 3:
				if (currentAngle < 295 || currentAngle > 305){ // rotate to around 300 degrees
					drive.mecanumDrive_Cartesian(0, 0, -0.75 * turnScalar, 0); // rotate left at 75% speed
				}
				else{
					autoState++;
					driveEncoder.reset();
				}
				break;
			case 4:
				if (driveEncoder.getDistance() < 42){
					drive.mecanumDrive_Cartesian(0, 0.75, 0, 0); // go forward at 75% speed
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
		case leftGearAuto:
			//go forward 78.5 inches
		switch (autoState){
			case 0:
				if (driveEncoder.getDistance() < 78.5){ 
					drive.mecanumDrive_Cartesian(0, 0.75, 0, 0); // go forward at 75% speed
				}
				else{
					autoState++;
				}
				break;
			case 1:
				if (currentAngle <= 60){ // turn 60 degrees
					drive.mecanumDrive_Cartesian(0, 0, 0.75 * turnScalar, 0); // rotate at 75% speed
				}
				else{
					autoState++;
					driveEncoder.reset();
				}
				break;
			case 2:
				if (driveEncoder.getDistance() < 25){
					drive.mecanumDrive_Cartesian(0, 0.75, 0, 0); // go forward at 75% speed
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
		case frontGearAuto:
			default:
				switch (autoState){
				
				case 0:
					// 1. Move forward
					if (driveEncoder.getDistance() < 68){ // The distance to go forward
						drive.mecanumDrive_Cartesian(0, 0.75, 0, 0); // go forward at 75% speed
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
		}
	}
	
	@Override
	public void testPeriodic() {
		
	}
}

