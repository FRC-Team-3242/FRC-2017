protected class BallPickup{
	
	private CANTalon pickup;
	private CANTalon elevator;
	
	public BallPickup(CANTalon pickup, CANTalon elevator){
		this.pickup = pickup;
		this.elevator = elevator;
	}
	
	public void pickupBall(){
		pickup.set(1);
		elevator.set(1); //Need to test speeds
	}
	
	public void stopPickup(){
		pickup.set(0);
		elevator.set(0);
	}
}

