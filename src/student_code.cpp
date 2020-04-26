#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
    // TODO Part 1.
      vector<Vector2D> intermediates = std::vector<Vector2D>(); 
      for (int i = 0; i < points.size() - 1; i++) {
          float int_x = points[i].x + (points[i + 1].x - points[i].x) * t; 
          float int_y = points[i].y + (points[i + 1].y - points[i].y) * t; 
          intermediates.push_back(Vector2D(int_x, int_y)); 
      }
      return intermediates;
  }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
      //exactly the same as Part 1 but in 3 dimensions
      vector<Vector3D> intermediates = std::vector<Vector3D>();
      for (int i = 0; i < points.size() - 1; i++) {
          float int_x = points[i].x + (points[i + 1].x - points[i].x) * t;
          float int_y = points[i].y + (points[i + 1].y - points[i].y) * t;
          float int_z = points[i].z + (points[i + 1].z - points[i].z) * t; 
          intermediates.push_back(Vector3D(int_x, int_y, int_z));
      }
      return intermediates;
  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
      std::vector<Vector3D> controls = points; 
      for (int i = 0; i < points.size() - 1; i++) {
          controls = BezierPatch::evaluateStep(controls, t); 
    }
      return controls[0];
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    // TODO Part 2.
      std::vector<Vector3D> crossPoints = std::vector<Vector3D>();
      for (int n = 0; n < controlPoints.size(); n++) {
          //cout << n << endl; 
          crossPoints.push_back(evaluate1D(controlPoints[n], u));
      }
      return evaluate1D(crossPoints, v);
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.
      std::vector<Vector3D> neighbors = std::vector<Vector3D>();
      HalfedgeCIter h = halfedge();
      do {
          neighbors.push_back(h->twin()->vertex()->position);
          h = h->twin()->next();
      }
      while (h != halfedge());
      int num_neighbors = neighbors.size(); 
      Vector3D myNorm = Vector3D(0., 0., 0.); 
      double total_weight = 0; 
      for (int i = 0; i < num_neighbors; i++) {
          //generate the two "vectors
          Vector3D v1 = neighbors[i] - position;
          Vector3D v2 = neighbors[(i + 1) % num_neighbors] - position;
          Vector3D crossv1v2 = -cross(v1, v2); 
          total_weight += 0.5 * crossv1v2.norm(); 
          myNorm += crossv1v2 * 0.5;
      }
      return myNorm / total_weight; 
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
      //e0->isNew = false; 
      if (e0 -> isBoundary()) {
          return e0; 
      }
      HalfedgeIter h_a1 = e0->halfedge();
      HalfedgeIter h_a2 = h_a1->next();
      HalfedgeIter h_a3 = h_a2->next();

      HalfedgeIter h_b1 = h_a1->twin();
      HalfedgeIter h_b2 = h_b1->next();
      HalfedgeIter h_b3 = h_b2->next();
      //Based on my drawing, there is no need to update any twins
      
      //Update next values. 
      h_a1->next() = h_b3; 
      h_a2->next() = h_a1;
      h_a3->next() = h_b2;
      h_b1->next() = h_a3;
      h_b2->next() = h_b1; 
      h_b3->next() = h_a2; 

      //Update vertices
      h_a1->vertex() = h_a3->vertex(); 
      h_b1->vertex() = h_b3->vertex();
      h_b2->vertex()->halfedge() = h_b2;
      h_a2->vertex()->halfedge() = h_a2; 
      
      //Update face
      h_a1->face()->halfedge() = h_a1; 
      h_b1->face()->halfedge() = h_b1; 
      h_a3->face() = h_b1->face(); 
      h_b3->face() = h_a1->face(); 

      //Update edge
      //they're the fuCKIN' SAMEEEEE
      return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
      if (e0->isBoundary()) {
          return e0->halfedge()->vertex();
      }
      //Define Friends
      HalfedgeIter bc = e0->halfedge();
      HalfedgeIter ca = bc->next();
      HalfedgeIter ab = ca->next();
      HalfedgeIter cb = bc->twin();
      HalfedgeIter bd = cb->next(); 
      HalfedgeIter dc = bd->next(); 
      VertexIter a = ab->vertex(); 
      VertexIter b = bd->vertex();
      VertexIter c = ca->vertex();
      VertexIter d = dc->vertex(); 
      //a->isNew = false;
      //b->isNew = false;
      //c->isNew = false; 
      //d->isNew = false;
      FaceIter f0 = ca->face(); 
      FaceIter f1 = dc->face(); 

      //make m
      VertexIter m = newVertex();
      m->position = 0.5 * (bc->vertex()->position + cb->vertex()->position);
      m->isNew = true; 

      //Introducing new halfedges
      HalfedgeIter am = newHalfedge(); 
      HalfedgeIter mc = newHalfedge(); 
      HalfedgeIter cm = cb;
      HalfedgeIter md = newHalfedge(); 
      HalfedgeIter bm = bc; 
      HalfedgeIter ma = newHalfedge(); 
      HalfedgeIter dm = newHalfedge();
      HalfedgeIter mb = newHalfedge(); 

      //Introducing new faces
      FaceIter f2 = newFace(); 
      FaceIter f3 = newFace(); 
      f0->halfedge() = ca; 
      f1->halfedge() = dc; 
      f2->halfedge() = bd;
      f3->halfedge() = ab; 

      //Introducing new edges
      EdgeIter e1 = newEdge(); 
      EdgeIter e2 = newEdge();
      EdgeIter e3 = newEdge();
      //Updating edges
      e0->halfedge() = mc; 
      e1->halfedge() = md; 
      e2->halfedge() = mb; 
      e3->halfedge() = ma; 
      e1->isNew = true; 
      e3->isNew = true; 
      e0->isNew = false;
      e2->isNew = false; 

      //Update vertices
      a->halfedge() = am;
      b->halfedge() = bm; 
      c->halfedge() = cm; 
      d->halfedge() = dm; 

      //Updating all of this jazz should be fun
      //F0
      //    Next
      mc->next() = ca; 
      ca->next() = am; 
      am->next() = mc; 
      //    Twin
      mc->twin() = cm; 
      //ca->twin() = ac; 
      am->twin() = ma; 
      //    Face
      mc->face() = f0; 
      ca->face() = f0; 
      am->face() = f0; 
      //    Vertices
      mc->vertex() = m; 
      ca->vertex() = c; 
      am->vertex() = a; 
      //    Edges
      mc->edge() = e0;
      am->edge() = e3; 
      //ca doesn't matter

      //F1
      //    Next
      cm->next() = md; 
      md->next() = dc; 
      dc->next() = cm; 
      //    Twin
      cm->twin() = mc; 
      md->twin() = dm; 
      //dc->twin() = cd; 
      //    Face
      cm->face() = f1; 
      md->face() = f1; 
      dc->face() = f1; 
      //    Vertices
      cm->vertex() = c; 
      md->vertex() = m; 
      dc->vertex() = d; 
      //    Edges
      cm->edge() = e0; 
      md->edge() = e1; 
      //dc edge doesn't matter

      //F2
      //    Next
      mb->next() = bd; 
      bd->next() = dm; 
      dm->next() = mb; 
      //    Twin
      mb->twin() = bm; 
      //skip bd
      dm->twin() = md; 
      //    Face
      mb->face() = f2; 
      bd->face() = f2; 
      dm->face() = f2; 
      //    Vertices
      mb->vertex() = m; 
      bd->vertex() = b; 
      dm->vertex() = d; 
      //    Edges
      mb->edge() = e2; 
      //skip bd
      dm->edge() = e1; 

      //F3
      //    Next
      bm->next() = ma; 
      ma->next() = ab; 
      ab->next() = bm; 
      //    Twin
      bm->twin() = mb; 
      ma->twin() = am; 
      //skip ab
      //    Face
      bm->face() = f3; 
      ma->face() = f3; 
      ab->face() = f3; 
      //    Vertices
      bm->vertex() = b; 
      ma->vertex() = m; 
      ab->vertex() = a; 
      //    Edges
      bm->edge() = e2; 
      ma->edge() = e3; 
      //skip ab
      
      m->halfedge() = mc;
   
    return m;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.

      //Precompute new positions
      cout << "1" << endl; 
      for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
            VertexIter A = e->halfedge()->vertex();
            VertexIter B = e->halfedge()->twin()->vertex();
            VertexIter C = e->halfedge()->next()->next()->vertex();
            VertexIter D = e->halfedge()->twin()->next()->next()->vertex();
            e->newPosition = 3. / 8. * (A->position + B->position) + 1. / 8. * (C->position + D->position);
            e->isNew = false;
      }
      cout << "2" << endl; 
      for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
          int n = v->degree(); 
          float u;
          Vector3D original_neighbor_position_sum;
          if (n == 3) {
              u = 3. / 16.; 
          }
          else {
              u = 3. / (8. * (float)n);
          }
          HalfedgeIter h = v->halfedge()->twin();
          do
          {
              // don't count boundary loops
              if (!h->face()->isBoundary())
              {
                  original_neighbor_position_sum += h->vertex()->position;
              }

              // move to the next halfedge around the vertex
              h = h->next()->twin();
          } while (h != v->halfedge()->twin()); // done iterating over halfedges
          v->newPosition = (1 - (float)n * u) * v->position + u * original_neighbor_position_sum;
          v->isNew = false;
      }

      cout << "3" << endl;
    //4-1 Subdivision
    //  Split original
      int mesh_size = mesh.nEdges(); 
      EdgeIter edg = mesh.edgesBegin();
      for (int i = 0; i < mesh_size; i++) {
          mesh.splitEdge(edg)->position = edg->newPosition; //also updates position
          advance(edg,1); 
      }
    //  Flip New
      cout << "4" << endl; 
      for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
          if (e->isNew && (e->halfedge()->vertex()->isNew != e->halfedge()->twin()->vertex()->isNew)) {
              mesh.flipEdge(e); 
          }
      }

      cout << "5" << endl; 
      //Update old
      for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
          if (!v->isNew) {
              cout << "gotem" << endl; 
              v->position = v->newPosition; 
          }
      }
  }
}