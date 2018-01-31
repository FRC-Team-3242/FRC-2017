
package org.usfirst.frc.team3242.robot;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;

public class LEDs {

	private Timer timer;
	private Spark ledController;
	
	public LEDs(Spark spark) {
		timer = new Timer();
		ledController = spark;
	}

	
	public void setRainbow() {
		ledController.set(-0.99);
	}
	
	public void setPartyRainbow() {
		ledController.set(-0.97);
	}
	
	
	public void setGlitterRainbow() {
		ledController.set(-0.89);
	}
	
	
}