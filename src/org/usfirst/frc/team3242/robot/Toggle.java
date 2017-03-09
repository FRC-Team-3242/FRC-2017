package org.usfirst.frc.team3242.robot;

import edu.wpi.first.wpilibj.Timer;

/**
 * uses a timer to ensure that toggles don't flip back and forth when
 * a button is held down
 *
 */
public class Toggle {
	private final double minWait = 0.2;
	private Timer timer;
	private boolean on;
	
	public Toggle(){
		this(false);
	}
	
	public Toggle(boolean defaultValue){
		timer = new Timer();
		timer.start();
		on = defaultValue;
	}
	
	public void toggle(boolean toggle){
		if(toggle) toggle();
	}
	
	public void toggle(){
		if(timer.get() > minWait){
			on = !on;
			timer.reset();
		}
	}
	
	public boolean getStatus(){
		return on;
	}
}
