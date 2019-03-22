package dissertation;

import javax.vecmath.Matrix4f;
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
	public int mass = 1;
	public float scale = 1;
	private Object3D object3D;
	private boolean growing = true;
	private RigidBody rigidBody;
	
	public Bacteria(DiscreteDynamicsWorld dynamicsWorld,World world) {
		initObject3D(world);
		initRigidBody(dynamicsWorld);
		Simulation.getBacteriaList().listIterator().add(this);
//		this.setMotionState(this);
	}

	public void initRigidBody(DiscreteDynamicsWorld dynamicsWorld ) {
		/* setup the Physics side */
		CollisionShape bacteriaShape = new CapsuleShape(7,2);
		DefaultMotionState fallMotionState = new DefaultMotionState(new Transform(new Matrix4f(new Quat4f(0, 0, 0, 1), new Vector3f(0, 10, 1), 1.0f)));
		Vector3f fallInertia = new Vector3f(0,0,0); 
		bacteriaShape.calculateLocalInertia(1,fallInertia);  
		RigidBodyConstructionInfo fallRigidBodyCI = new RigidBodyConstructionInfo(mass,fallMotionState,bacteriaShape,fallInertia); 
		rigidBody = new RigidBody(fallRigidBodyCI);
		dynamicsWorld.addRigidBody(rigidBody); //adds to the physics simulation		
	}

	public void initObject3D(World world) {
		object3D = Primitives.getEllipsoid(7, 2);
		object3D.setEnvmapped(Object3D.ENVMAP_ENABLED);
		object3D.setCollisionMode(Object3D.COLLISION_CHECK_SELF);
		object3D.setTexture("bacteria");
		world.addObject(object3D);
		object3D.build();	//builds the mesh for the object 
	}
	
	public void setMotionState(Bacteria bacteria) {
		JPCTMotionState msj;
		Transform trans = new Transform();
		bacteria.getRigidBody().getMotionState().getWorldTransform(trans);  
		msj = new JPCTMotionState(bacteria.getObject3D(), trans);
		bacteria.getObject3D().rotateZ((float) (Math.PI / 2f));
//		System.out.println("bacteria height: " + trans.origin.y); 
	}

	public float getScale() {
		return scale;
	}

	public void setScale(float scale) {
		this.scale = scale;
	}
	
	public boolean isGrowing() {
		return growing;
	}

	public void setGrowing(boolean growing) {
		this.growing = growing;
	}
	public RigidBody getRigidBody() {
		return rigidBody;
	}
	public Object3D getObject3D() {
		return object3D;
	}

}
