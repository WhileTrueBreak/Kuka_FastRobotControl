package jadevep.client.kinematics;

import org.ejml.simple.SimpleMatrix;

public class RefPlaneResult {
	public SimpleMatrix planeVector;
	public SimpleMatrix elbowRot;
	public double[] joints;
	public RefPlaneResult(SimpleMatrix planeVector, SimpleMatrix elbowRot, double[] joints) {
		this.planeVector = planeVector;
		this.elbowRot = elbowRot;
		this.joints = joints;
	}
}
