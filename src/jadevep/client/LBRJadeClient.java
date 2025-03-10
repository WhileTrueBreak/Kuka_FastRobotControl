package jadevep.client;
import com.kuka.connectivity.fastRobotInterface.clientSDK.clientLBR.LBRClient;
import jadevep.utils.Utils;

public class LBRJadeClient extends LBRClient{
	
	private final int NUM_JOINTS = 7;
	
	private boolean is_paired = false;

	private double[] joint_waypoint = {0,0,0,0,0,0,0};
	
	private double[][] joint_limits = {{-170, 170},{-120,120},{-170,170},{-120,120},{-170,170},{-120,120},{-175,175}};
	private double joint_limit_buffer = 0.005;
	
	private PIDController[] joint_controllers = new PIDController[NUM_JOINTS];
	private double max_joint_inc = 0.01;
	
	public LBRJadeClient(){
    	for(int i = 0;i < NUM_JOINTS;i++) {
    		joint_limits[i][0] = Math.toRadians(joint_limits[i][0])+joint_limit_buffer;
    		joint_limits[i][1] = Math.toRadians(joint_limits[i][1])-joint_limit_buffer;
    	}
    	for(int i = 0;i < NUM_JOINTS;i++) {
    		joint_controllers[i] = new PIDController(0.1, 0, 0);
    	}
	}
	
    @Override
    public void onStateChange(FRISessionState oldState, FRISessionState newState){
    	System.out.println("Changed from " + oldState.toString() + " to " + newState.toString());
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
    	this.clamp_joints(this.joint_waypoint);
    	this.getRobotCommand().setJointPosition(this.joint_waypoint);
    }
    
    private void update() {
    	if(!this.is_paired) {
    		double[] measured_joints = this.getRobotState().getMeasuredJointPosition();
    		for(int i = 0;i < NUM_JOINTS;i++) {
        		joint_controllers[i].setSetpoint(measured_joints[i]);
        	}
    		this.is_paired = true;
    	}
    	double[] current_joints = this.getRobotState().getMeasuredJointPosition();
    	double[] joint_incs = new double[NUM_JOINTS];
    	for(int  i = 0;i < this.NUM_JOINTS;i++) {
    		joint_incs[i] = joint_controllers[i].update(current_joints[i]);
    		joint_incs[i] = Utils.clamp(joint_incs[i], -max_joint_inc, max_joint_inc);
    	}
    	
    	for(int  i = 0;i < this.NUM_JOINTS;i++) {
    		this.joint_waypoint[i] = current_joints[i]+joint_incs[i];
    	}
    	System.out.print("next: ");
    	print_joints(joint_waypoint);
    }
    
    private void clamp_joints(double[] joints) {
    	for(int i = 0;i < this.joint_limits.length;i++) {
    		joints[i] = Utils.clamp(joints[i], this.joint_limits[i][0], this.joint_limits[i][1]);
    	}
    }
    
    public void print_joints(double[] joints) {
    	for(int i = 0 ;i < joints.length;i++) {
    		System.out.print((double) Math.round(joints[i]*1000)/1000 + " | ");
    	}
        System.out.println();
    }
    
    public void setTargetJoints(double[] target) {
    	for(int i = 0;i < NUM_JOINTS;i++) {
    		joint_controllers[i].setSetpoint(target[i]);
    	}
    }
    
	public void setMax_joint_inc(double max_joint_inc) {
		this.max_joint_inc = max_joint_inc;
	}
	
	public void setPIDK(double kp, double ki, double kd) {
		for(int i = 0;i < NUM_JOINTS;i++) {
    		joint_controllers[i].setKp(kp);
    		joint_controllers[i].setKi(ki);
    		joint_controllers[i].setKd(kd);
    	}
	}
	
}
