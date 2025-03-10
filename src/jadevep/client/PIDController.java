package jadevep.client;

public class PIDController {

	private boolean first_update;
	
	private long last_time;
	
	private double err_total, last_err;
	
	private double kp, ki, kd;
	private double setpoint;
	
	public PIDController(double kp, double ki, double kd) {
		this.kp = kp;
		this.ki = ki;
		this.kd = kd;
		this.setpoint = 0;
		this.err_total = 0;
		this.last_err = 0;
		this.first_update = true;
	}
	
	public double update(double input) {
		
		long now = System.nanoTime();
		double delta_time = (double)(now - last_time)/1000000000;
		double err = setpoint - input;
		err_total += err * delta_time;
		double delta_err = (err - last_err)/delta_time;

		last_time = now;
		last_err = err;
		
		if(first_update) {
			err_total = 0;
			first_update = false;
			return 0;
		}
		
		return kp * err + ki * err_total + kd * delta_err;
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
