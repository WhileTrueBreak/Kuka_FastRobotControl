package jadevep.client.kinematics;

import org.ejml.simple.SimpleMatrix;

public class FKResult {
	
	public SimpleMatrix pose;
	public double nsParam;
	public Configuration robotConf;
	
	public FKResult(SimpleMatrix pose, double nsParam, Configuration robotConf) {
		this.pose = pose;
		this.nsParam = nsParam;
		this.robotConf = robotConf;
	}
	
}
