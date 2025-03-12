package jadevep.client.kinematics;

public class TargetIKReturn {
	public boolean isReachable;
	public boolean isJointExceed;
	public double[] jointExceedErrs;
	public double[] distFromTargets;
	
	private TargetIKReturn(boolean isReachable, boolean isJointExceed, double[] jointExceedErrs, double[] distFromTargets) {
		this.isReachable = isReachable;
		this.isJointExceed = isJointExceed;
		this.jointExceedErrs = jointExceedErrs;
		this.distFromTargets = distFromTargets;
	}
	
	public static TargetIKReturn returnUnreachable() {
		return new TargetIKReturn(false, false, null, null);
	}
	
	public static TargetIKReturn jointExceed(double[] jointErrs) {
		return new TargetIKReturn(false, true, jointErrs, null);
	}
	
	public static TargetIKReturn reachable(double[] distFromTargets) {
		return new TargetIKReturn(true, false, null, distFromTargets);
	}
	
}
