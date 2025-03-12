package jadevep.client;

import com.kuka.connectivity.fastRobotInterface.clientSDK.base.ClientApplication;
import com.kuka.connectivity.fastRobotInterface.clientSDK.connection.UdpConnection;

public class LBRJadeClientThread extends Thread{

	private boolean isRunning;
	
	private String hostname = "172.32.1.149";
	private int port = 30200;
	
	private LBRJadeClient client;
	private ClientApplication app;
	
	public LBRJadeClientThread(String hostname, int port) {
		this.hostname = hostname;
		this.port = port;
		
		this.client = new LBRJadeClient();
		UdpConnection connection = new UdpConnection();
		this.app = new ClientApplication(connection, this.client);
		this.app.connect(this.port, this.hostname);
	}
	
	@Override
	public void run() {
		System.out.println("Starting client thread...");
		this.isRunning = true;
        
        long startTime = System.nanoTime();
        long lastTime = startTime;
        int stepCount = 0;
        
        boolean success = true;
        while (success && this.isRunning){
        	try {
        		success = this.app.step();
        		stepCount++;
        		
        		long now = System.nanoTime();
                if ((now - lastTime) >= 1_000_000_000L) { // Every second
                    double elapsedSec = (now - lastTime) / 1_000_000_000.0;
                    double stepRate = stepCount / elapsedSec;
                    System.out.println("Step Rate: " + stepRate + " steps/sec");

                    lastTime = now;
                    stepCount = 0;
                }
        	} catch (Exception e) {
        		e.printStackTrace();
        		break;
			}
        }
        stopClient();
		System.out.println("Stopped client thread.");
	}

	public void stopClient() {
		if(!this.isRunning) return;
		System.out.println("Stopping client thread...");
		this.isRunning = false;
		app.disconnect();
	}
	
	public LBRJadeClient getClient() {
		return this.client;
	}
	
	public boolean isRunning() {
		return this.isRunning;
	}
	
}
