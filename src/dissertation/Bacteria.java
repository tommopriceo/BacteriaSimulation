package dissertation;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Matrix4f;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;

import com.bulletphysics.collision.shapes.CapsuleShape;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.Transform;
import com.threed.jpct.Object3D;
import com.threed.jpct.Primitives;
import com.threed.jpct.World;

public class Bacteria {
	public int mass = 1;
	public float scale = 1;
	private static List<Object3D> bacteriaList = new ArrayList<Object3D>(); 
	private static List<RigidBody> rigidBodyList = new ArrayList<RigidBody>();
	private boolean growing = true;
	public RigidBody getBacteriaPhysics(DiscreteDynamicsWorld dynamicsWorld){
		/* setup the Physics side */
		CollisionShape bacteriaShape = new CapsuleShape(6,3);
		DefaultMotionState fallMotionState = new DefaultMotionState(new Transform(new Matrix4f(new Quat4f(0, 0, 0, 1), new Vector3f(0, 8, 1), 1.0f)));
		Vector3f fallInertia = new Vector3f(0,0,0); 
		bacteriaShape.calculateLocalInertia(10000,fallInertia);  
		RigidBodyConstructionInfo fallRigidBodyCI = new RigidBodyConstructionInfo(mass,fallMotionState,bacteriaShape,fallInertia); 
		RigidBody fallRigidBody = new RigidBody(fallRigidBodyCI);
		dynamicsWorld.addRigidBody(fallRigidBody); //adds to the physics simulation
		rigidBodyList.add(fallRigidBody);
		return fallRigidBody;	
	}
	
	public Object3D getBacteriaGraphics(World world) {
		Object3D bacteria3D = Primitives.getEllipsoid(6, 3);
		bacteria3D.setEnvmapped(Object3D.ENVMAP_ENABLED);
		bacteria3D.setCollisionMode(Object3D.COLLISION_CHECK_SELF);
		bacteria3D.setTexture("bacteria");
		world.addObject(bacteria3D);
		bacteria3D.rotateZ((float) (Math.PI / 2f));
		bacteria3D.build();
		bacteriaList.add(bacteria3D);	
		return bacteria3D;
	}

	public void setMotionState(RigidBody rigidBody) {
		JPCTMotionState msj;
		Bacteria bacteriaHelper = new Bacteria();
		Transform trans = new Transform();
		rigidBody.getMotionState().getWorldTransform(trans);  
		msj = new JPCTMotionState(bacteriaHelper.getBacteriaList().get(0), trans); //need to make this dynamic 
		System.out.println("bacteria height: " + trans.origin.y); 
	}
	
	public  List<Object3D> getBacteriaList() {
		return bacteriaList;
	}

	public List<RigidBody> getRigidBodyList() {
		return rigidBodyList;
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

}
