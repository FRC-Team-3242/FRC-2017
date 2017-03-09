package org.usfirst.frc.team3242.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Spark;

public class GearDropper {
	private Spark dropperActuator;
	private AnalogInput potentiometer;
	private final double maxVal = 1.9;//2.1618
	private final double minVal = 1.7;//1.545
	private final double speed = 0.3;
	
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
	
	public void set(double s){
		if(Math.abs(s) > 0.1)
			dropperActuator.set(s*0.5);
	}
	
	public void set(boolean extend, boolean retract){
		if(extend){
			set(-speed);
		}else if(retract){
			set(speed);
		}else{
			set(0);
		}
	}
	
	public void open(boolean open){
		if(open && potentiometer.getValue() > minVal){
			set(false, true);//retract
		}else if(potentiometer.getValue() < maxVal){
			set(true, false);//extend
		}else{
			set(false, false);
		}
	}
	
}
