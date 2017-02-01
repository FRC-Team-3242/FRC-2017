package org.usfirst.frc.team3242.robot;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.filters.LinearDigitalFilter;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class VisionServer {
	private NetworkTable table;
	private boolean found;
	private double x;
	private double y;
	public final VisionCoordinate xCoordRaw;
	public final VisionCoordinate yCoordRaw;
	public final LinearDigitalFilter xFiltered;
	public final LinearDigitalFilter yFiltered;
	private Timer timeoutTimer;
	private Timer updateTimer;
	private final double updateTime = 0.0333333333;//30 times per second (camera framerate)
	private final double timeoutTime = 2;
	
	/**
	 * retrieves coordinate data from raspberry pi over network tables
	 * use 'xCoord' and 'yCoord' class variables to retrieve coordinate data
	 */
	public VisionServer(){
		table = NetworkTable.getTable("rpi");
		found = false;
		x = -1;
		y = -1;
		xCoordRaw = new VisionCoordinate(true);
		yCoordRaw = new VisionCoordinate(false);
		//15 taps should be half a second
		xFiltered = LinearDigitalFilter.movingAverage(xCoordRaw, 15);
		yFiltered = LinearDigitalFilter.movingAverage(yCoordRaw, 15);
		timeoutTimer = new Timer();
		timeoutTimer.start();
		updateTimer = new Timer();
		updateTimer.start();
	}
	
	public boolean targetFound(){
		return found;
	}
	
	
	/**
	 * @return if a target has been identified
	 */
	private boolean update(){
		found = table.getBoolean("found", false);
    	if(found){
        	x = table.getNumber("visionX", -1);
    		y = table.getNumber("visionY", -1);
    	}
		updateTimer.reset();
		if(timeoutTimer.get() > timeoutTime){
			xFiltered.reset();
			yFiltered.reset();
		}
		timeoutTimer.reset();
    	return found;
	}
	
	private void checkTime(){
		if(updateTimer.get() >= updateTime){
			update();
		}
	}
	
	public class VisionCoordinate implements PIDSource{
		
		boolean isX;
		
		/**
		 * This class takes the coordinate information from the raspberry pi to
		 * create a PIDSource that can be used in filters and PID loops
		 * 
		 * @param isX determines whether this object is the x or y coordinate
		 */
		public VisionCoordinate(boolean isX){
			this.isX = isX;
		}
		
		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {
			//do nothing, only use displacement
		}

		@Override
		public PIDSourceType getPIDSourceType() {
			return PIDSourceType.kDisplacement;
		}

		@Override
		public double pidGet() {
			if(isX){
				return x;
			}else{
				return y;
			}
		}
	}
}
