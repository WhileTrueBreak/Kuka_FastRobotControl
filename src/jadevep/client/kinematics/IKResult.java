package jadevep.client.kinematics;

import org.ejml.simple.SimpleMatrix;

public class IKResult {
	public double[] joints;
	public SimpleMatrix[] sMat;
	public SimpleMatrix[] wMat;
	
	public IKResult(double[] joints, SimpleMatrix[] sMat, SimpleMatrix[] wMat) {
		this.joints = joints;
		this.sMat = sMat;
		this.wMat = wMat;
	}
}
