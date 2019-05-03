package dissertation;

import javax.vecmath.Matrix4f;

import java.util.Random;
import java.util.concurrent.ThreadLocalRandom;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import com.bulletphysics.collision.shapes.CapsuleShape;
import com.bulletphysics.collision.shapes.CapsuleShapeX;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.Transform;
import com.threed.jpct.Object3D;
import com.threed.jpct.Primitives;
import com.threed.jpct.SimpleVector;
import com.threed.jpct.World;

public class Bacteria {
	private float mass = 1f;
	private float scale = 1;
	private Object3D object3D;
	private RigidBody rigidBody;
	private double maxScale = 1.0021800;
	
	public Bacteria(DiscreteDynamicsWorld dynamicsWorld,World world) {
		initObject3D(world);
		initRigidBody(dynamicsWorld);
		Simulation.getBacteriaList().listIterator().add(this);
	}

	public void initRigidBody(DiscreteDynamicsWorld dynamicsWorld ) {
		/* setup the Physics side */
		CollisionShape bacteriaShape = new CapsuleShape(3.5f,2);
		DefaultMotionState fallMotionState = new DefaultMotionState(new Transform(new Matrix4f(new Quat4f(0, 0, 0, 1), new Vector3f(0, 4.5f, 0), 1.0f)));
		Vector3f fallInertia = new Vector3f(0,0,0); 
		bacteriaShape.calculateLocalInertia(0,fallInertia);  
		RigidBodyConstructionInfo fallRigidBodyCI = new RigidBodyConstructionInfo(mass,fallMotionState,bacteriaShape,fallInertia); 
		rigidBody = new RigidBody(fallRigidBodyCI);
		rigidBody.setFriction(15f);  
		dynamicsWorld.addRigidBody(rigidBody); //adds to the physics simulation		
		
	}

	public void initObject3D(World world) {
		object3D = Primitives.getEllipsoid(3.5f, 2);
		object3D.setCollisionMode(Object3D.COLLISION_CHECK_SELF);
		object3D.setTexture("bacteria");
		object3D.setSpecularLighting(true); //This sets 3D lighting affects 
		object3D.setEnvmapped(Object3D.ENVMAP_ENABLED);   //This handles reflections and what not
		world.addObject(object3D);
		object3D.build();	//builds the mesh for the object 
	}
	
	public void setMotionState(Bacteria bacteria) {
		JPCTMotionState msj;
		Transform trans = new Transform();
		bacteria.getRigidBody().getMotionState().getWorldTransform(trans);  
		msj = new JPCTMotionState(bacteria.getObject3D(), trans);
		bacteria.getObject3D().rotateZ((float) (Math.PI / 2f));
	}

	public float getScale() {
		return scale;
	}

	public void setScale(float scale) {
		this.scale = scale;
	}
	
	public RigidBody getRigidBody() {
		return rigidBody;
	}
	public Object3D getObject3D() {
		return object3D;
	}
	
	public double getMaxScale() {
		return maxScale;
	}

	public void setMaxScale(double maxScale) {
		this.maxScale = maxScale;
	}

}
