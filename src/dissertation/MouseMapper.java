package dissertation;
import java.awt.event.KeyEvent;

import org.lwjgl.input.Mouse;
import com.threed.jpct.FrameBuffer;
import com.threed.jpct.Matrix;
import com.threed.jpct.SimpleVector;
import com.threed.jpct.util.KeyMapper;
import com.threed.jpct.util.KeyState;

class MouseMapper {
	
	private boolean hidden = false;
	private int height = 0;
	private static boolean doLoop = true;
	static KeyMapper keyMapper = new KeyMapper();
	static boolean forward = false; 
 	static boolean backward = false;
	static boolean up = false;
	static boolean down = false;
	static boolean left = false;
	static boolean right = false;
	static float xAngle = 0;


public  MouseMapper(FrameBuffer buffer) {
	height = buffer.getOutputHeight();
	init();
}

public void hide() {
	if (!hidden) {
		Mouse.setGrabbed(true);
		hidden = true;
	}
}

public void show() {
	if (hidden) {
		Mouse.setGrabbed(false);
		hidden = false;
	}
}

public boolean isVisible() {
	return !hidden;
}

public void destroy() {
	show();
	if (Mouse.isCreated()) {
		Mouse.destroy();
	}
}

public boolean buttonDown(int button) {
	return Mouse.isButtonDown(button);
}

public int getMouseX() {
	return Mouse.getX();
}

public int getMouseY() {
	return height - Mouse.getY();
}

public int getDeltaX() {
	if (Mouse.isGrabbed()) {
		return Mouse.getDX();
	} else {
		return 0;
	}
}

public int getDeltaY() {
	if (Mouse.isGrabbed()) {
		return Mouse.getDY();
	} else {
		return 0;
	}
}

private void init() {
	try {
		if (!Mouse.isCreated()) {
			Mouse.create();
		}

	} catch (Exception e) {
		throw new RuntimeException(e);
	}
}


}