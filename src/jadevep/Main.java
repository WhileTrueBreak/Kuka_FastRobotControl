package jadevep;

import jadevep.inputs.KeyListener;

public class Main {

	public static void main(String[] args) {
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
