package jadevep.client;
import org.ejml.simple.SimpleMatrix;

import com.kuka.connectivity.fastRobotInterface.clientSDK.clientLBR.LBRClient;

import jadevep.client.kinematics.Configuration;
import jadevep.client.kinematics.FKResult;
import jadevep.client.kinematics.IKResult;
import jadevep.client.kinematics.Kinematics;
import jadevep.client.kinematics.KinematicsHelper;
import jadevep.client.kinematics.TargetIKReturn;
import jadevep.utils.Utils;

public class LBRJadeClient extends LBRClient{
	
	private static final double TOL = 1e-8;
	
	private final int NUM_JOINTS = 7;
	
	private boolean isPaired = false;

	private ClientControlMode controlMode;

	private SimpleMatrix targetPose = null;
	private Configuration targetConf = null;
	private double targetRAxis = 0;
	
	private double[] jointWaypoint = {0,0,0,0,0,0,0};
	
	private double[][] jointLimits = {{-170, 170},{-120,120},{-170,170},{-120,120},{-170,170},{-120,120},{-175,175}};
	private double jointLimitBuffer = 0.005;
	
	private PIDController[] jointControllers = new PIDController[NUM_JOINTS];
	private double maxJointInc = 0.01;
	private double maxCartInc = 0.005;
	
	private FRISessionState currentState = FRISessionState.IDLE;
	
	public LBRJadeClient(){
		controlMode = ClientControlMode.JOINT;
    	for(int i = 0;i < NUM_JOINTS;i++) {
    		jointLimits[i][0] = Math.toRadians(jointLimits[i][0])+jointLimitBuffer;
    		jointLimits[i][1] = Math.toRadians(jointLimits[i][1])-jointLimitBuffer;
    	}
    	for(int i = 0;i < NUM_JOINTS;i++) {
    		jointControllers[i] = new PIDController(0.1, 0, 0);
    	}
	}
	
    @Override
    public void onStateChange(FRISessionState oldState, FRISessionState newState){
    	System.out.println("Changed from " + oldState.toString() + " to " + newState.toString());
    	currentState = newState;
    }
    
    @Override
    public void monitor() {
    	super.monitor();
    	this.update();
    }
    
    @Override
    public void waitForCommand() {
    	super.waitForCommand();
    	this.update();
    }
    
    @Override
    public void command() {
    	this.update();
    	this.clampJoints(this.jointWaypoint);
    	this.getRobotCommand().setJointPosition(this.jointWaypoint);
    }
    
    private void update() {
    	if(!this.isPaired) {
    		double[] measuredJoints = this.getRobotState().getMeasuredJointPosition();
    		for(int i = 0;i < NUM_JOINTS;i++) {
        		jointControllers[i].setSetpoint(measuredJoints[i]);
        		jointWaypoint[i] = measuredJoints[i];
        	}
    		
    		this.isPaired = true;
    	}
    	switch (controlMode) {
		case JOINT:
			this.jointControlUpdate();
			break;
		case CARTESIAN:
			this.cartesianControlUpdate();
			break;
		default:
			break;
		}
    	System.out.print("Waypoint: ");
    	printJoints(jointWaypoint);
    }
    
    private void cartesianControlUpdate() {
    	if(this.targetPose == null) return;
    	FKResult currFkResult = this.getCurrentFK();
    	if(currFkResult == null) return;
    	System.out.println(this.targetPose);
    	double currPoseX = currFkResult.pose.get(0,3);
    	double currPoseY = currFkResult.pose.get(1,3);
    	double currPoseZ = currFkResult.pose.get(2,3);
    	double targPoseX = this.targetPose.get(0,3);
    	double targPoseY = this.targetPose.get(1,3);
    	double targPoseZ = this.targetPose.get(2,3);
    	double poseStepX = Utils.clamp(targPoseX-currPoseX, -maxCartInc, maxCartInc);
    	double poseStepY = Utils.clamp(targPoseY-currPoseY, -maxCartInc, maxCartInc);
    	double poseStepZ = Utils.clamp(targPoseZ-currPoseZ, -maxCartInc, maxCartInc);
    	SimpleMatrix waypointPose = this.targetPose.copy();
    	waypointPose.set(0, 3, currPoseX+poseStepX);
    	waypointPose.set(1, 3, currPoseY+poseStepY);
    	waypointPose.set(2, 3, currPoseZ+poseStepZ);
    	System.out.printf("%.3f, %.3f, %.3f\n", poseStepX, poseStepY, poseStepZ);
    	try {
    		IKResult waypointIK = Kinematics.InverseKinematics(waypointPose, this.targetRAxis, this.targetConf);
    		for(int i = 0;i < waypointIK.joints.length;i++) {
    			if(waypointIK.joints[i] < this.jointLimits[i][0] || waypointIK.joints[i] > this.jointLimits[i][1]) return;
    		}
        	double[] currentJoints = this.getRobotState().getMeasuredJointPosition();
        	double[] jointIncs = new double[NUM_JOINTS];
        	double maxInc = 0;
        	for(int  i = 0;i < this.NUM_JOINTS;i++) {
        		jointIncs[i] = waypointIK.joints[i] - currentJoints[i];
        		if(Math.abs(jointIncs[i]) > maxInc) {
        			maxInc = Math.abs(jointIncs[i]);
        		}
        	}
        	double adjustRatio = maxJointInc/maxInc;
        	if(adjustRatio < 1) {
	        	for(int  i = 0;i < this.NUM_JOINTS;i++) {
	        		jointIncs[i] *= adjustRatio;
	        	}
        	}

        	for(int  i = 0;i < this.NUM_JOINTS;i++) {
        		this.jointWaypoint[i] = currentJoints[i]+jointIncs[i];
        	}
    	}catch (Exception e) {
    		return;
		}
    }
    
    private void jointControlUpdate() {
    	double[] currentJoints = this.getRobotState().getMeasuredJointPosition();
    	double[] jointIncs = new double[NUM_JOINTS];
    	for(int  i = 0;i < this.NUM_JOINTS;i++) {
    		jointIncs[i] = jointControllers[i].update(currentJoints[i]);
    		jointIncs[i] = Utils.clamp(jointIncs[i], -this.maxJointInc, this.maxJointInc);
    	}
    	
    	for(int  i = 0;i < this.NUM_JOINTS;i++) {
    		this.jointWaypoint[i] = currentJoints[i]+jointIncs[i];
    	}
    }
    
    private void clampJoints(double[] joints) {
    	for(int i = 0;i < this.jointLimits.length;i++) {
    		joints[i] = Utils.clamp(joints[i], this.jointLimits[i][0], this.jointLimits[i][1]);
    	}
    }
    
    public void printJoints(double[] joints) {
    	for(int i = 0 ;i < joints.length;i++) {
    		System.out.print((double) Math.round(joints[i]*1000)/1000 + " | ");
    	}
        System.out.println();
    }
    
    public void setTargetJoints(double[] target) {
    	for(int i = 0;i < NUM_JOINTS;i++) {
    		jointControllers[i].setSetpoint(target[i]);
    	}
    }
    
    public TargetIKReturn setTargetPose(double x, double y, double z, double a, double b, double c, double r, Configuration robotConf) {
    	SimpleMatrix rotX = KinematicsHelper.rotationMatrixX(c);
    	SimpleMatrix rotY = KinematicsHelper.rotationMatrixY(b);
    	SimpleMatrix rotZ = KinematicsHelper.rotationMatrixZ(a);
    	SimpleMatrix pose = rotX.mult(rotY).mult(rotZ);
    	pose.set(0, 3, x);
    	pose.set(1, 3, y);
    	pose.set(2, 3, z);
    	return this.setTargetPose(pose, r, robotConf);
    }
    
    public TargetIKReturn setTargetPose(SimpleMatrix pose, double r, Configuration robotConf) {
    	try {
    		IKResult ikResult 	= Kinematics.InverseKinematics(pose, r, robotConf);
    		double[] clamped = ikResult.joints.clone();
    		this.clampJoints(clamped);
    		double[] jointLimitErrs = new double[clamped.length];
    		double[] jointTargetErrs = new double[clamped.length];
    		boolean isOutOfBounds = false;
    		for(int i = 0;i < clamped.length;i++) {
    			jointLimitErrs[i] = Math.abs(clamped[i]-ikResult.joints[i]);
    			if(jointLimitErrs[i] > TOL) isOutOfBounds = true;
    			jointTargetErrs[i] = Math.abs(jointControllers[i].getSetpoint()-ikResult.joints[i]);
    		}
    		if(isOutOfBounds) return TargetIKReturn.jointExceed(jointTargetErrs);
    		this.setTargetJoints(ikResult.joints);
    		this.targetPose = pose;
    		this.targetConf = robotConf;
    		this.targetRAxis = r;
    		return TargetIKReturn.reachable(jointTargetErrs);
    	}catch(Exception e){
    		return TargetIKReturn.returnUnreachable();
    	}
    }
    
    public FKResult getCurrentFK() {
    	if(currentState == FRISessionState.IDLE) return null;
    	try {
	    	double[] measuredJoints = this.getRobotState().getMeasuredJointPosition();
	    	return Kinematics.ForwardKinematics(measuredJoints);
    	}catch (Exception e) {
			return null;
		}
    }
    
 	public void setMaxJointInc(double maxJointInc) {
		this.maxJointInc = maxJointInc;
	}
	
	public void setPIDK(double kp, double ki, double kd) {
		for(int i = 0;i < NUM_JOINTS;i++) {
    		jointControllers[i].setKp(kp);
    		jointControllers[i].setKi(ki);
    		jointControllers[i].setKd(kd);
    	}
	}

	public void setCartesianMode() {
		controlMode = ClientControlMode.CARTESIAN;
	}
	
	public void setJointMode() {
		controlMode = ClientControlMode.JOINT;
	}
	
}
