package jadevep.inputs;

import java.util.concurrent.ConcurrentHashMap;
import java.util.logging.Level;
import java.util.logging.Logger;

import com.github.kwhat.jnativehook.GlobalScreen;
import com.github.kwhat.jnativehook.keyboard.NativeKeyEvent;
import com.github.kwhat.jnativehook.keyboard.NativeKeyListener;

public class KeyListener implements NativeKeyListener {
	
	private static KeyListener keyListener = null;
	
    private static final ConcurrentHashMap<String, Boolean> keyStates = new ConcurrentHashMap<>();
    
    private KeyListener() {
        System.out.println("Starting key listener...");
        try {
            GlobalScreen.registerNativeHook();
        } catch (Exception e) {
            e.printStackTrace();
        }
        GlobalScreen.addNativeKeyListener(this);

        // Disable JNativeHook logging to prevent console spam
        Logger logger = Logger.getLogger(GlobalScreen.class.getPackage().getName());
        logger.setLevel(Level.OFF);
    }

    @Override
    public void nativeKeyPressed(NativeKeyEvent e) {
        keyStates.put(NativeKeyEvent.getKeyText(e.getKeyCode()), true);
    }

    @Override
    public void nativeKeyReleased(NativeKeyEvent e) {
        keyStates.put(NativeKeyEvent.getKeyText(e.getKeyCode()), false);
    }

    @Override
    public void nativeKeyTyped(NativeKeyEvent e) {
        // Not used
    }
    
    public static boolean isKeyPressed(String keyCode) {
        return keyStates.getOrDefault(keyCode, false);
    }
    
    public static void initListener() {
    	if (keyListener != null) return;
    	keyListener = new KeyListener();
    }
    
    public static void stopListener() {
        try {
            GlobalScreen.unregisterNativeHook();
            keyListener = null;
            System.out.println("Stopped key listener.");
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
    
}
