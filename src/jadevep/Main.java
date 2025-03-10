package jadevep;

import jadevep.client.LBRJadeClient;
import jadevep.client.LBRJadeClientThread;
import jadevep.inputs.KeyListener;
import jadevep.utils.PIDController;

public class Main {

	public static void main(String[] args) throws InterruptedException {
		
//		PIDController c = new PIDController(0.005, 0.00005, 0.005);
//		c.setSetpoint(0);
//		double p = 2*Math.PI;
//		for(int i = 0;i < 200;i++) {
//			double change = c.update(p);
//			System.out.println(p);
//			p += change;
//			Thread.sleep(10);
//		}

		KeyListener.initListener();
		String hostname = "172.32.1.149";
		int port = 30200;
		
		LBRJadeClientThread clientThread = new LBRJadeClientThread(hostname, port);
		LBRJadeClient client = clientThread.getClient();
		clientThread.setPriority(Thread.MAX_PRIORITY);
		clientThread.start();
		
		while(true) {
			if(KeyListener.isKeyPressed("P")||!clientThread.isAlive()) {
				break;
			}
			if(KeyListener.isKeyPressed("A")) {
				double[] j = {Math.PI/2,0,0,Math.PI/2,0,-Math.PI/2,0};
				client.setTargetJoints(j);
			}
			if(KeyListener.isKeyPressed("S")) {
				double[] j = {0,0,0,0,0,0,0};
				client.setTargetJoints(j);
			}
		}
		
		clientThread.stopClient();
        KeyListener.stopListener();
	}

}
