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
	
	private final int NUM_JOINTS = 7;
	
	private boolean isPaired = false;

	private double[] jointWaypoint = {0,0,0,0,0,0,0};
	
	private double[][] jointLimits = {{-170, 170},{-120,120},{-170,170},{-120,120},{-170,170},{-120,120},{-175,175}};
	private double jointLimitBuffer = 0.005;
	
	private PIDController[] jointControllers = new PIDController[NUM_JOINTS];
	private double maxJointInc = 0.01;
	
	private FRISessionState currentState = FRISessionState.IDLE;
	
	public LBRJadeClient(){
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
        	}
    		this.isPaired = true;
    	}
    	double[] currentJoints = this.getRobotState().getMeasuredJointPosition();
    	double[] jointIncs = new double[NUM_JOINTS];
    	for(int  i = 0;i < this.NUM_JOINTS;i++) {
    		jointIncs[i] = jointControllers[i].update(currentJoints[i]);
    		jointIncs[i] = Utils.clamp(jointIncs[i], -this.maxJointInc, this.maxJointInc);
    	}
    	
    	for(int  i = 0;i < this.NUM_JOINTS;i++) {
    		this.jointWaypoint[i] = currentJoints[i]+jointIncs[i];
    	}
    	System.out.print("next: ");
    	printJoints(jointWaypoint);
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
    	double tol = 1e-8;
    	try {
    		IKResult ikResult 	= Kinematics.InverseKinematics(pose, r, robotConf);
    		double[] clamped = ikResult.joints.clone();
    		this.clampJoints(clamped);
    		double[] jointLimitErrs = new double[clamped.length];
    		double[] jointTargetErrs = new double[clamped.length];
    		boolean isOutOfBounds = false;
    		for(int i = 0;i < clamped.length;i++) {
    			jointLimitErrs[i] = Math.abs(clamped[i]-ikResult.joints[i]);
    			if(jointLimitErrs[i] > tol) isOutOfBounds = true;
    			jointTargetErrs[i] = Math.abs(jointControllers[i].getSetpoint()-ikResult.joints[i]);
    		}
    		if(isOutOfBounds) return TargetIKReturn.jointExceed(jointTargetErrs);
    		this.setTargetJoints(ikResult.joints);
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
	
}
