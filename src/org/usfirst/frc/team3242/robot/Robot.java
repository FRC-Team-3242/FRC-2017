package org.usfirst.frc.team3242.robot;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.PigeonImu;
import com.ctre.PigeonImu.FusionStatus;

/**
 * If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	final String frontGearAuto = "Front Gear";
	final String leftGearAuto = "Left Gear";
	final String rightGearAuto = "Right Gear";
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();
	
	Encoder driveEncoder;
	RobotDrive drive;
	PigeonImu imu;
	PigeonImu.GeneralStatus genStatus;
	boolean turnOne;
	boolean turnTwo;
	
	double[] ypr = new double[3];

	
	@Override
	public void robotInit() {
		chooser.addDefault("Default Auto", frontGearAuto);
		chooser.addObject("Left Gear", leftGearAuto);
		chooser.addObject("Right Gear", rightGearAuto);
		SmartDashboard.putData("Auto choices", chooser);
		
		driveEncoder = new Encoder(0, 1, false, CounterBase.EncodingType.k4X); 
		driveEncoder.setDistancePerPulse(0.014);
		
		drive = new RobotDrive(new CANTalon(0), new CANTalon(1), new CANTalon(2), new CANTalon(3));
		
		genStatus = new PigeonImu.GeneralStatus();
		
		imu = new PigeonImu(0);
		
		imu.GetGeneralStatus(genStatus);
		
		turnOne = false;
		turnTwo = false;
		
	}
	
	@Override
	public void autonomousInit() {
		autoSelected = chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
	}

	@Override
	public void autonomousPeriodic() {
		switch (autoSelected) {
		case rightGearAuto:
			//go forward 12 inches
			if (driveEncoder.getDistance() < 12 && !turnOne){ 
				drive.mecanumDrive_Cartesian(0, 0.75, 0, 0); // go forward at 75% speed
				break;
			}
			imu.GetYawPitchRoll(ypr);
			if (ypr[0] <= 30){ // turn 30 degrees
				drive.mecanumDrive_Cartesian(0, 0, 0.75, 0); // rotate right at 75% speed
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
			
			if (ypr[0] < 295 || ypr[0] > 305 && !turnTwo){
				drive.mecanumDrive_Cartesian(0, 0, -0.75, 0); // rotate left at 75% speed
				break;
			}
			
			if (!turnTwo){
				turnTwo = true;
				driveEncoder.reset();
			}
			
			if (driveEncoder.getDistance() < 20 && turnTwo){
				drive.mecanumDrive_Cartesian(0, 0.75, 0, 0); // go forward at 75% speed
				break;
			}
			
			// to_do: use auto gear placing function
			break;
		case leftGearAuto:
			//go forward 78.5 inches
			if (driveEncoder.getDistance() < 78.5 && !turnOne){ 
				drive.mecanumDrive_Cartesian(0, 0.75, 0, 0); // go forward at 75% speed
				break;
			}
			imu.GetYawPitchRoll(ypr);
			if (ypr[0] <= 60){ // turn 60 degrees
				drive.mecanumDrive_Cartesian(0, 0, 0.75, 0); // rotate at 75% speed
				break;
			}
			if (!turnOne){
				turnOne = true;
				driveEncoder.reset();
			}

			if (driveEncoder.getDistance() < 25 && turnOne){
				drive.mecanumDrive_Cartesian(0, 0.75, 0, 0); // go forward at 75% speed
			}
			break;
			
			// to_do: use auto gear placing function
			
		case frontGearAuto:
			default:
				// 1. Move forward
				if (driveEncoder.getDistance() < 68){ // The distance to go forward
					drive.mecanumDrive_Cartesian(0, 0.75, 0, 0); // go forward at 75% speed
				}
				// to_do: 2. Use auto gear placing function
				//
				break;
			
		}
	}

	@Override
	public void teleopPeriodic() {
	}

	
	@Override
	public void testPeriodic() {
	}
}

