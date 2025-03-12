package jadevep.client;

public class PIDController {

	private boolean firstUpdate;
	
	private long lastTime;
	
	private double errTotal, lastErr;
	
	private double kp, ki, kd;
	private double setpoint;
	
	public PIDController(double kp, double ki, double kd) {
		this.kp = kp;
		this.ki = ki;
		this.kd = kd;
		this.setpoint = 0;
		this.errTotal = 0;
		this.lastErr = 0;
		this.firstUpdate = true;
	}
	
	public double update(double input) {
		
		long now = System.nanoTime();
		double deltaTime = (double)(now - lastTime)/1000000000;
		double err = setpoint - input;
		errTotal += err * deltaTime;
		double deltaErr = (err - lastErr)/deltaTime;

		lastTime = now;
		lastErr = err;
		
		System.out.println("err_t: "+errTotal);
		
		if(firstUpdate) {
			errTotal = 0;
			firstUpdate = false;
			return 0;
		}
		
		return kp * err + ki * errTotal + kd * deltaErr;
	}

	public double getKp() {
		return kp;
	}

	public void setKp(double kp) {
		this.kp = kp;
	}

	public double getKi() {
		return ki;
	}

	public void setKi(double ki) {
		this.ki = ki;
	}

	public double getKd() {
		return kd;
	}

	public void setKd(double kd) {
		this.kd = kd;
	}

	public double getSetpoint() {
		return setpoint;
	}

	public void setSetpoint(double setpoint) {
		this.setpoint = setpoint;
	}
	
}
