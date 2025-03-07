package jadevep;
import com.kuka.connectivity.fastRobotInterface.clientSDK.clientLBR.LBRClient;
import jadevep.inputs.KeyListener;
import jadevep.utils.Utils;

public class LBRJadeClient extends LBRClient{
	
	private final int NUM_JOINTS = 7;
	
	private boolean is_paired = false;
	
	private double[] joint_waypoint = {0,0,0,0,0,0,0};
	private double[] joint_target = {0,0,0,0,0,0,0};
	
	private double[][] joint_limits = {{-170, 170},{-120,120},{-170,170},{-120,120},{-170,170},{-120,120},{-175,175}};
	private double joint_limit_buffer = 0.005;
	
	private double joint_speed = Math.PI/2;
	
	public LBRJadeClient(){
    	for(int i = 0;i < joint_limits.length;i++) {
    		joint_limits[i][0] = Math.toRadians(joint_limits[i][0])+joint_limit_buffer;
    		joint_limits[i][1] = Math.toRadians(joint_limits[i][1])-joint_limit_buffer;
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
    		this.joint_target = this.getRobotState().getMeasuredJointPosition();
    		this.is_paired = true;
    	}
    	double[] current_joints = this.getRobotState().getMeasuredJointPosition();
//    	double joint_speed_inc = this.joint_speed * this.getRobotState().getSampleTime();
    	double joint_speed_inc = 0.007;
    	double[] joint_change = {0,0,0,0,0,0,0};
    	for(int  i = 0;i < this.NUM_JOINTS;i++) {
    		joint_change[i] = Utils.clamp(this.joint_target[i] - current_joints[i], -joint_speed_inc, joint_speed_inc);
    		this.joint_waypoint[i] = current_joints[i]+Utils.clamp(this.joint_target[i] - current_joints[i], -joint_speed_inc, joint_speed_inc);
    	}
    	System.out.print("next: ");
    	printJoints(joint_waypoint);
    }
    
    public void clamp_joints(double[] joints) {
    	for(int i = 0;i < this.joint_limits.length;i++) {
    		joints[i] = Utils.clamp(joints[i], this.joint_limits[i][0], this.joint_limits[i][1]);
    	}
    }
    
    public void printJoints(double[] joints) {
    	for(int i = 0 ;i < joints.length;i++) {
    		System.out.print((double) Math.round(joints[i]*1000)/1000 + " | ");
    	}
        System.out.println();
    }
    
    public void setTargetJoints(double[] target) {
    	this.joint_target = target;
    }
}
