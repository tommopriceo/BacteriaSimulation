package dissertation;

import javax.vecmath.Matrix4f;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.Transform;
import com.threed.jpct.Object3D;
import com.threed.jpct.Primitives;
import com.threed.jpct.World;

public class WorldShapes {
	
	public static void createBox(DiscreteDynamicsWorld dynamicsWorld, World world, Vector3f position) {
		//This has to be called after the world has been created or will result in null pointer
		Object3D box = initObject3DBox(world);
		RigidBody boxBody = initRigidBodyBox(dynamicsWorld);
		boxBody.translate(position);
		JPCTMotionState boxMS;
		Transform trans = new Transform();
		boxBody.getWorldTransform(trans);  
		boxMS = new JPCTMotionState(box, trans);
	}
	
	
	public static RigidBody initRigidBodyBox(DiscreteDynamicsWorld dynamicsWorld ) {
		CollisionShape boxShape = new BoxShape(new Vector3f(10f,5f,1));
		DefaultMotionState boxMotionState = new DefaultMotionState(new Transform(new Matrix4f(new Quat4f(0, -50, 0, 1), new Vector3f(0, -1, 0), 1.0f))); 
		RigidBodyConstructionInfo rbInfo = new RigidBodyConstructionInfo(0, boxMotionState, boxShape, new Vector3f(0,0,0));
		RigidBody  boxBody = new RigidBody(rbInfo);
		boxBody.setFriction(10f);
		dynamicsWorld.addRigidBody(boxBody);	
		return boxBody;
	}
	
	public static Object3D initObject3DBox(World world) {
		Object3D box = Primitives.getBox(5f, 5f);
		box.translate(40, -20, 0);
		box.setTexture("box");
		box.setEnvmapped(Object3D.ENVMAP_ENABLED);
		box.setCollisionMode(Object3D.COLLISION_CHECK_OTHERS);
		world.addObject(box);
		return box;
	}
	
}
