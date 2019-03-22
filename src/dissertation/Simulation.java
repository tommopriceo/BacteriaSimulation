package dissertation;

import java.awt.event.KeyEvent;
import java.util.ArrayList;
import java.util.List;

import javax.vecmath.*;

import com.threed.jpct.*;
import com.threed.jpct.util.KeyMapper;
import com.threed.jpct.util.KeyState;
import com.bulletphysics.collision.broadphase.BroadphaseInterface;
import com.bulletphysics.collision.broadphase.DbvtBroadphase;
import com.bulletphysics.collision.dispatch.CollisionDispatcher;
import com.bulletphysics.collision.dispatch.DefaultCollisionConfiguration;
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

public class Simulation {
	
	private static FrameBuffer buffer;
	public int maxSubSteps;
	public float timeStep, fixedTimeStep;
	private static World world, sky;
	private static Object3D plane, dome;
	public static float scale = 1;
	public static double maxScale = 1.0031800;
	public static int numBoxes = 10;
	public static boolean growing = false;
	protected static Clock clock = new Clock(); 
	private static List<Bacteria> bacteriaList = new ArrayList<Bacteria>();
	static boolean forward = false;
 	static boolean backward = false;
	static boolean up = false;
	static boolean down = false;
	static boolean left = false;
	static boolean right = false;
	static Ticker ticker = new Ticker(15);

	public static void main(String[] args) {
		
		/* Settings */
//		Config.glAvoidTextureCopies = true;
		Config.maxPolysVisible = 100000;
		Config.glColorDepth = 24;
		Config.glFullscreen = false;
		Config.farPlane = 4000;
		Config.glShadowZBias = 0.8f;
		Config.lightMul = 1;
		Config.collideOffset = 0;
		Config.glTrilinear = true;
		
		/*Textures*/
		TextureManager tm = TextureManager.getInstance();
		tm.addTexture("sky", new Texture("/Users/yungtommo/eclipse-workspace/dissertation/src/dissertation/textures/sky.jpg"));
		tm.addTexture("bacteria", new Texture("/Users/yungtommo/eclipse-workspace/dissertation/src/dissertation/textures/bacteria.jpg"));
		tm.addTexture("plane", new Texture("/Users/yungtommo/eclipse-workspace/dissertation/src/dissertation/textures/floor.jpg"));
		
		/* Create the physical world using JBullet*/
		BroadphaseInterface broadphase = new DbvtBroadphase();
		DefaultCollisionConfiguration collisionConfiguration = new DefaultCollisionConfiguration();
		CollisionDispatcher dispatcher = new CollisionDispatcher(collisionConfiguration);
		SequentialImpulseConstraintSolver solver = new SequentialImpulseConstraintSolver();
		DiscreteDynamicsWorld dynamicsWorld = new DiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
		
		/* set the gravity of world*/
		dynamicsWorld.setGravity(new Vector3f(0, -10, 0));

		/* setup collision shapes */
		CollisionShape groundShape = new StaticPlaneShape(new Vector3f(0, 1, 0), 1);
	
		/* setup ground motion state */
		DefaultMotionState groundMotionState = new DefaultMotionState(new Transform(new Matrix4f(new Quat4f(0, 0, 0, 1), new Vector3f(0, -1, 0), 1.0f))); 
		RigidBodyConstructionInfo groundRigidBodyCI = new RigidBodyConstructionInfo(0, groundMotionState, groundShape, new Vector3f(0,0,0)); 
		RigidBody groundRigidBody = new RigidBody(groundRigidBodyCI); 
		dynamicsWorld.addRigidBody(groundRigidBody); // add ground 

		/* Create FrameBuffer to display */
		buffer = new FrameBuffer(800, 600, FrameBuffer.SAMPLINGMODE_NORMAL);
		buffer.disableRenderer(IRenderer.RENDERER_SOFTWARE);
		buffer.enableRenderer(IRenderer.RENDERER_OPENGL);
		
		/* Create Visual world */
		world = new World();
		world.setAmbientLight(255, 255, 255);
		
		
		
		/*Create Objects*/
		plane = Primitives.getPlane(100, 100); //Sets the size of the floor
		plane.rotateX((float) (Math.PI / 2f));  //Sets the orientation  
		plane.setTexture("plane");
		plane.setCollisionMode(Object3D.COLLISION_CHECK_OTHERS); // other objects may collide with this object
//		plane.setSpecularLighting(true); //This sets 3D lighting affects 
//		plane.setEnvmapped(Object3D.ENVMAP_ENABLED);   //This handles reflections and what not
		

		/* Add the objects into the world */
		world.addObject(plane);  //adds the floor to the 
		world.buildAllObjects();


		Camera cam = world.getCamera();
		cam.moveCamera(Camera.CAMERA_MOVEOUT, 90);
		cam.moveCamera(Camera.CAMERA_MOVEUP, 20);
		cam.lookAt(plane.getTransformedCenter());
		cam.setFOV(1.5f);

		plane.compileAndStrip();
		KeyMapper keyMapper = new KeyMapper();
		Bacteria bacteria = new Bacteria(dynamicsWorld, world);
		new Bacteria(dynamicsWorld, world).getRigidBody().translate(new Vector3f(40,0,10));
//		bacteria1.getRigidBody().translate(new Vector3f(40,0,10));
//		Bacteria bacteria1 = new Bacteria(dynamicsWorld, world);

	
//		bacteria1.getRigidBody().translate(new Vector3f(50,0,0));
		
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
				if (bac.getScale() <= maxScale) {
					VertexController vertexController = new VertexController(bac3D);
					vertexController.scale(new SimpleVector(1f,scale,1f));
					bacRig.getCollisionShape().setLocalScaling(new Vector3f(1f, scale, 1f));
					bac.setScale(scale);
				}else {
					Transform position = new Transform(); 
					bacRig.getWorldTransform(position);
					System.out.println(position.getMatrix(new Matrix4f()));
					world.removeObject(bac3D);
					dynamicsWorld.removeRigidBody(bacRig);
					Simulation.getBacteriaList().remove(bac);
				    Bacteria b1 = new Bacteria(dynamicsWorld, world);
				    Bacteria b2 = new Bacteria(dynamicsWorld, world);
				    b1.getRigidBody().setWorldTransform(position);
//				    System.out.println(position.toString());
					scale = 1;
				}
				bac3D.rebuild();
				bac.setMotionState(bac);
			}
			float ms = clock.getTimeMicroseconds();
			clock.reset();
			dynamicsWorld.stepSimulation(ms / 1000000f);
			scale = scale + 0.000010f;
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