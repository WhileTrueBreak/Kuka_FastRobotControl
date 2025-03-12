package jadevep;

import org.ejml.simple.SimpleMatrix;

import jadevep.client.LBRJadeClient;
import jadevep.client.LBRJadeClientThread;
import jadevep.client.kinematics.Configuration;
import jadevep.client.kinematics.FKResult;
import jadevep.inputs.KeyListener;

public class Main {

	public static void main(String[] args) throws InterruptedException {
		KeyListener.initListener();
		String hostname = "172.24.201.1";
		int port = 30200;
		
		LBRJadeClientThread clientThread = new LBRJadeClientThread(hostname, port);
		LBRJadeClient client = clientThread.getClient();
		clientThread.setPriority(Thread.MAX_PRIORITY);
		clientThread.start();
		
		SimpleMatrix movePose = null;
		Configuration moveConf = null;
		double moveRAxis = 0;
		
		while(true) {
			if(KeyListener.isKeyPressed("P")||!clientThread.isAlive()) break;
			if(movePose == null) {
				FKResult fkResult = client.getCurrentFK();
				if(fkResult == null) continue;
				movePose = fkResult.pose;
				moveConf = fkResult.robotConf;
				moveRAxis = fkResult.nsParam;
			}

			if(KeyListener.isKeyPressed("W")) {
				movePose.set(0, 3, movePose.get(0, 3)+0.00001);
			}
			if(KeyListener.isKeyPressed("S")) {
				movePose.set(0, 3, movePose.get(0, 3)-0.00001);
			}
			
			client.setTargetPose(movePose, moveRAxis, moveConf);
		}
		
		clientThread.stopClient();
		KeyListener.stopListener();
	}

}
