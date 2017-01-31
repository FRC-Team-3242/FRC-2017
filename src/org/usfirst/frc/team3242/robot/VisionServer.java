package org.usfirst.frc.team3242.robot;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class VisionServer {
	private NetworkTable table;
	private boolean found;
	private double x;
	private double y;
	public final VisionCoordinate xCoord;
	public final VisionCoordinate yCoord;
	private Timer timer;
	//30 times per second (camera framerate)
	private final double updateTime = 0.0333333333;
	
	/**
	 * retrieves coordinate data from raspberry pi over network tables
	 * use 'xCoord' and 'yCoord' class variables to retrieve coordinate data
	 */
	public VisionServer(){
		table = NetworkTable.getTable("rpi");
		found = false;
		x = -1;
		y = -1;
		xCoord = new VisionCoordinate(true);
		yCoord = new VisionCoordinate(false);
		timer = new Timer();
		timer.start();
	}
	
	/**
	 * @return if a target has been identified
	 */
	public boolean update(){
		found = table.getBoolean("found", false);
    	if(found){
        	x = table.getNumber("visionX", -1);
    		y = table.getNumber("visionY", -1);
    	}
		timer.reset();
    	return found;
	}
	
	private void checkTime(){
		if(timer.get() >= updateTime){
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
				checkTime();
				return x;
			}else{
				return y;
			}
		}
	}
}
