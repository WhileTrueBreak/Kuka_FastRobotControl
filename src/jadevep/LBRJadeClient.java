package jadevep;
import java.util.Arrays;
import com.kuka.connectivity.fastRobotInterface.clientSDK.clientLBR.LBRClient;
import jadevep.inputs.KeyListener;

public class LBRJadeClient extends LBRClient{
	
	double time_elapsed = 0;
	double[] joint_waypoints = new double[7];
	
	public LBRJadeClient(){
		Arrays.fill(joint_waypoints, 0);
		joint_waypoints[0] = Math.PI/2;
		joint_waypoints[3] = Math.PI/2;
		joint_waypoints[5] = -Math.PI/2;
	}
    
    @Override
    public void onStateChange(FRISessionState oldState, FRISessionState newState){
    	System.out.println("Changed from " + oldState.toString() + " to " + newState.toString());
        switch (newState){
        case MONITORING_READY:
        case IDLE:
        case MONITORING_WAIT:
        default:
            break;
        }
    }
    
    public void check_keys() {
    	int[] keys = {16,17,18,19,20,21,22,
    			      30,31,32,33,34,35,36};
    	for(int i = 0;i < 7;i++) {
    		if(KeyListener.isKeyPressed(keys[i])) joint_waypoints[i] += 0.003;
    		if(KeyListener.isKeyPressed(keys[i+7])) joint_waypoints[i] -= 0.003;
    	}
    }
    
    @Override
    public void command() {
    	check_keys();
    	printJoints(joint_waypoints);
    	getRobotCommand().setJointPosition(joint_waypoints);
    }
    
    public void printJoints(double[] joints) {
    	for(int i = 0 ;i < joints.length;i++) {
    		System.out.print((double) Math.round(joints[i]*1000)/1000 + " | ");
    	}
        System.out.println();
    }
}
