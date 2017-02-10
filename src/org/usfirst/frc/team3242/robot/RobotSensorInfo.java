package org.usfirst.frc.team3242.robot;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.PigeonImu;
import com.ctre.PigeonImu.FusionStatus;

public class RobotSensorInfo{
	public RobotSensorInfo(){}
	
	public static double normalizeIMUAngle(PigeonImu imu){
		imu.GetGeneralStatus(new PigeonImu.GeneralStatus());
		double currentAngle = imu.GetFusedHeading(new PigeonImu.FusionStatus());
		return currentAngle % 360;
	}
}