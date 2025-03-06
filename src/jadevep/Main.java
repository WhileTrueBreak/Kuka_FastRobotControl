package jadevep;

import com.kuka.connectivity.fastRobotInterface.clientSDK.base.ClientApplication;
import com.kuka.connectivity.fastRobotInterface.clientSDK.connection.UdpConnection;

import jadevep.inputs.KeyListener;

public class Main {

	public static void main(String[] args) {
		KeyListener.initListener();
		
		String hostname = "172.32.1.149";
		int port = 30200;
		
		LBRJadeClient client = new LBRJadeClient();
		UdpConnection connection = new UdpConnection();
        ClientApplication app = new ClientApplication(connection, client);
        app.connect(port, hostname);
        
        boolean success = true;
        while (success){
        	try {
        		success = app.step();
        	}catch (Exception e) {
        		e.printStackTrace();
        		break;
			}
        }
        
        app.disconnect();
        KeyListener.stopListener();
	}

}
