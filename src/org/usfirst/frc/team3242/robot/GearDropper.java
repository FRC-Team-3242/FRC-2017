package org.usfirst.frc.team3242.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Spark;

public class GearDropper {
	private Spark dropperActuator;
	private AnalogInput potentiometer;
	private final double maxVal = 0;
	private final double minVal = 13;
	private final double speed = 0.8;
	
	public GearDropper(Spark actuator, AnalogInput pot){
		this.dropperActuator = actuator;
		this.potentiometer = pot;
	}
	
	public double getAvgPotentiometerVal(){
		return potentiometer.getAverageVoltage();
	}
	
	public double getPotentiometerVal(){
		return potentiometer.getVoltage();
	}
	
	public void override(boolean extend, boolean retract){
		if(extend){
			override(speed);
		}else if(retract){
			override(-speed);
		}else{
			override(0);
		}
	}
	
	public void override(double s){
		dropperActuator.set(s);
	}
	
	public void open(boolean open){
		if(open && potentiometer.getValue() < maxVal){
			dropperActuator.set(speed);
		}else if(potentiometer.getValue() > minVal){
			dropperActuator.set(-speed);
		}
	}
	
}
