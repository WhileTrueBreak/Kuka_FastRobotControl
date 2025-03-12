package jadevep.client.kinematics;

public class Configuration {
	
	private boolean arm, elbow, wrist;
	
	public Configuration(boolean arm, boolean elbow, boolean wrist) {
		this.arm = arm;
		this.elbow = elbow;
		this.wrist = wrist;
	}
	
	public int getArm() {
		if(arm) return 1;
		return -1;
	}
	
	public int getElbow() {
		if(elbow) return 1;
		return -1;
	}
	
	public int getWrist() {
		if(wrist) return 1;
		return -1; 
	}

	public void setArm(boolean arm) {
		this.arm = arm;
	}

	public void setElbow(boolean elbow) {
		this.elbow = elbow;
	}

	public void setWrist(boolean wrist) {
		this.wrist = wrist;
	}
	
}
