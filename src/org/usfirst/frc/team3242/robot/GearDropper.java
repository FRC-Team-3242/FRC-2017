package org.usfirst.frc.team3242.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Spark;

public class GearDropper {
	private Spark dropperActuator;
	private AnalogInput potentiometer;
	private final double maxVal = 11;
	private final double minVal = 2;
	private final double speed = 0.8;
	
	public GearDropper(Spark actuator, AnalogInput pot){
		this.dropperActuator = actuator;
		this.potentiometer = pot;
	}
	
	public void open(boolean open){
		if(open && potentiometer.getValue() < maxVal){
			dropperActuator.set(speed);
		}else if(potentiometer.getValue() > minVal){
			dropperActuator.set(-speed);
		}
	}
	
}
