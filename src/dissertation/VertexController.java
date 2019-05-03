package dissertation;

import com.threed.jpct.GenericVertexController;
import com.threed.jpct.Object3D;
import com.threed.jpct.SimpleVector;

class VertexController extends GenericVertexController 
{
   public VertexController(Object3D toCheck) { 
       super.init(toCheck.getMesh(), true);
   }

   protected void scale(SimpleVector scale) {
      SimpleVector[] vertices = getSourceMesh();
      SimpleVector[] destination = getDestinationMesh();

      for (int i = 0; i < vertices.length; i++) 
      {
         vertices[i].x *= scale.x;
         vertices[i].y *= scale.y;
         vertices[i].z *= scale.z;
         destination[i].x = vertices[i].x;
         destination[i].y = vertices[i].y;
         destination[i].z = vertices[i].z;
      }

      this.updateMesh();
   }

   public void apply() {}
}