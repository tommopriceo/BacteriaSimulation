package dissertation;

import java.awt.event.KeyEvent;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.ThreadLocalRandom;

import javax.vecmath.*;

import com.threed.jpct.*;
import com.threed.jpct.util.KeyMapper;
import com.threed.jpct.util.KeyState;
import com.bulletphysics.collision.broadphase.BroadphaseInterface;
import com.bulletphysics.collision.broadphase.DbvtBroadphase;
import com.bulletphysics.collision.dispatch.CollisionDispatcher;
import com.bulletphysics.collision.dispatch.DefaultCollisionConfiguration;
import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.collision.shapes.CapsuleShape;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.StaticPlaneShape;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import com.bulletphysics.linearmath.Clock;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.Transform;

public class Simulation  {
	
	private static FrameBuffer buffer;
	private static World world;
	private static Object3D plane;
	private static float scale = 1;
	protected static Clock clock = new Clock(); 
	private static List<Bacteria> bacteriaList = new ArrayList<Bacteria>();
	private static boolean forward = false;
	private static boolean backward = false;
	private static boolean up = false;
	private static boolean down = false;
	private static boolean left = false;
	private static boolean right = false;
	private static Ticker ticker = new Ticker(15);
	
	/* Video Settings */
	private static int windowX = 800;
	private static int windowY = 600;
	private static Float camFOV = 5.5f;
	
	/* Scaling Probability */
	private static int max = 100;
	private static int min = 1;
	private static int threshold = 50;
	private static double maxScale = 1.0021000;

	/* World Settings */
	private static Vector3f gravity =  new Vector3f(0f,-20.8f,0f); //X,Y,Z
	private static Float stepTime = 0.01666f; //60hz
	private static Float floorFriction = 5f;
	private static int floorX = 20;
	private static int floorY = 30;
	
	public static void main(String[] args) {
		
		/* Renderer Settings */
		Config.glAvoidTextureCopies = true;
		Config.maxPolysVisible = 500000;
		Config.glColorDepth = 24;
		Config.glFullscreen = false;
		Config.farPlane = 4000;
		Config.glShadowZBias = 0.8f;
		Config.lightMul = 1;
		Config.collideOffset = 500;
		
		/* Buffer Settings */
		buffer = new FrameBuffer(windowX, windowY, FrameBuffer.SAMPLINGMODE_GL_AA_4X);
		buffer.disableRenderer(IRenderer.RENDERER_SOFTWARE);
		buffer.enableRenderer(IRenderer.RENDERER_OPENGL);
		
		/*Textures*/
		TextureManager tm = TextureManager.getInstance();
		tm.addTexture("bacteria", new Texture("./src/dissertation/textures/bacteria.jpg"));
		tm.addTexture("plane", new Texture("./src/dissertation/textures/whitesmall.jpg"));
		tm.addTexture("box", new Texture("./src/dissertation/textures/floor.jpg"));
		
		/* Create the physical world using JBullet*/
		BroadphaseInterface broadphase = new DbvtBroadphase();
		DefaultCollisionConfiguration collisionConfiguration = new DefaultCollisionConfiguration();
		CollisionDispatcher dispatcher = new CollisionDispatcher(collisionConfiguration);
		SequentialImpulseConstraintSolver solver = new SequentialImpulseConstraintSolver();
		DiscreteDynamicsWorld dynamicsWorld = new DiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
		
		/* set the gravity of world*/
		dynamicsWorld.setGravity(gravity);

		/* setup collision shapes */
		CollisionShape groundShape = new StaticPlaneShape(new Vector3f(0,1, 0), 1);
		/* setup ground physics */
		DefaultMotionState groundMotionState = new DefaultMotionState(new Transform(new Matrix4f(new Quat4f(0, 0, 0, 1), new Vector3f(0, -1, 0), 1.0f))); 
		RigidBodyConstructionInfo groundRigidBodyCI = new RigidBodyConstructionInfo(0, groundMotionState, groundShape, new Vector3f(0,0,0)); 
		RigidBody groundRigidBody = new RigidBody(groundRigidBodyCI); 
		groundRigidBody.setFriction(floorFriction);
		dynamicsWorld.addRigidBody(groundRigidBody); // add ground 

		/* Create Visual world */
		world = new World();
		world.setAmbientLight(255, 255, 255);
		
		/*Create Objects*/
		plane = Primitives.getPlane(floorX, floorY); //Sets the size of the floor
		plane.rotateX((float) (Math.PI / 2f));  //Sets the orientation  
		plane.setTexture("plane");
		plane.setCollisionMode(Object3D.COLLISION_CHECK_OTHERS); // other objects may collide with this object
		
		/* Add the objects into the world */
		world.addObject(plane);  //adds the floor to the 
		world.buildAllObjects();
	
		Camera cam = world.getCamera();
		cam.moveCamera(Camera.CAMERA_MOVEOUT, 90);
		cam.moveCamera(Camera.CAMERA_MOVEUP, 50);
		cam.lookAt(plane.getTransformedCenter());
		cam.setFOV(camFOV);
		
		KeyMapper keyMapper = new KeyMapper();
		Bacteria bacteria = new Bacteria(dynamicsWorld, world);
		
		while (!org.lwjgl.opengl.Display.isCloseRequested()) { 
			long ticks = 0;
			SimpleVector offset = new SimpleVector(1, 0, -1).normalize();
			ticks = ticker.getTicks();
			if (ticks > 0) {
				offset.rotateY(0.007f * ticks);
				pollControls(keyMapper);
				move(ticks);
			}
			for (Bacteria bac : new ArrayList<Bacteria>(Simulation.getBacteriaList())) {
				Object3D bac3D = bac.getObject3D();
				RigidBody bacRig = bac.getRigidBody();
				Random rand = new Random();
				int randomNum = rand.nextInt((max - min) + 1) + min;
//				if (randomNum >= threshold) {
					if (bac.getScale() <= maxScale) {
						VertexController vertexController = new VertexController(bac3D);
						vertexController.scale(new SimpleVector(1f,scale,1f));
						bacRig.getCollisionShape().setLocalScaling(new Vector3f(1f, scale, 1f));
						bac.setScale(scale);
					}else {
						
						Transform position1 = new Transform(); 
						Transform position2 = new Transform();
						bacRig.getWorldTransform(position1);
						bacRig.getWorldTransform(position2);
						position1.getMatrix(new Matrix4f());
						position2.getMatrix(new Matrix4f());
						dynamicsWorld.removeRigidBody(bacRig);
						world.removeObject(bac3D);
						Simulation.getBacteriaList().remove(bac);
					    Bacteria b1 = new Bacteria(dynamicsWorld, world);
					    Bacteria b2 = new Bacteria(dynamicsWorld, world);
					    b1.getRigidBody().setWorldTransform(position1);
					    b2.getRigidBody().setWorldTransform(position2);
					    b1.getRigidBody().translate(new Vector3f(-7f,0f,0f));
					    b2.getRigidBody().translate(new Vector3f(7f,0f,0f));	 	
					    scale = 1;
					}
				
				bac.setMotionState(bac);
			
//			}
			}
			dynamicsWorld.stepSimulation(stepTime);
//			System.out.println(Simulation.getBacteriaList().size());  //Shows how many bacteria are in the world
			scale = scale + 0.000004f;
			buffer.clear();
			world.renderScene(buffer);
			world.draw(buffer);
			buffer.update();
			buffer.displayGLOnly();

		}
		buffer.disableRenderer(IRenderer.RENDERER_OPENGL);
		buffer.dispose();
		System.exit(0);
	}

	/*Basic control functions - taken from JPTC advanced example*/
	private static void move(long ticks) { 

		if (ticks == 0) {
			return;
		}

		// Key controls
		SimpleVector ellipsoid = new SimpleVector(5, 5, 5);

		if (forward) {
			world.checkCameraCollisionEllipsoid(Camera.CAMERA_MOVEIN, 
					ellipsoid, ticks, 5);
		}

		if (backward) {
			world.checkCameraCollisionEllipsoid(Camera.CAMERA_MOVEOUT,
					ellipsoid, ticks, 5);
		}

		if (left) {
			world.checkCameraCollisionEllipsoid(Camera.CAMERA_MOVELEFT,
					ellipsoid, ticks, 5);
		}

		if (right) {
			world.checkCameraCollisionEllipsoid(Camera.CAMERA_MOVERIGHT,
					ellipsoid, ticks, 5);
		}

		if (up) {
			world.checkCameraCollisionEllipsoid(Camera.CAMERA_MOVEUP,
					ellipsoid, ticks, 5);
		}

		if (down) {
			world.checkCameraCollisionEllipsoid(Camera.CAMERA_MOVEDOWN,
					ellipsoid, ticks, 5);
		}

	}

	private static class Ticker {

		private int rate;
		private long s2;

		public static long getTime() {
			return System.currentTimeMillis();
		}

		public Ticker(int tickrateMS) {
			rate = tickrateMS;
			s2 = Ticker.getTime();
		}

		public int getTicks() {
			long i = Ticker.getTime();
			if (i - s2 > rate) {
				int ticks = (int) ((i - s2) / (long) rate);
				s2 += (long) rate * ticks;
				return ticks;
			}
			return 0;
		}
	}
	
	public static void pollControls(KeyMapper k) {
		KeyState ks = null;
		while ((ks = k.poll()) != KeyState.NONE) {
	
			if (ks.getKeyCode() == KeyEvent.VK_UP) {
				forward = ks.getState();
			}

			if (ks.getKeyCode() == KeyEvent.VK_DOWN) {
				backward = ks.getState();
			}

			if (ks.getKeyCode() == KeyEvent.VK_LEFT) {
				left = ks.getState();
			}

			if (ks.getKeyCode() == KeyEvent.VK_RIGHT) {
				right = ks.getState();
			}

			if (ks.getKeyCode() == KeyEvent.VK_PAGE_UP) {
				up = ks.getState();
			}

			if (ks.getKeyCode() == KeyEvent.VK_PAGE_DOWN) {
				down = ks.getState();
			}
		}
	}
	public static List<Bacteria> getBacteriaList() {
		return bacteriaList;
	}

	public static void setBacteriaList(List<Bacteria> bacteriaList) {
		Simulation.bacteriaList = bacteriaList;
	}

}