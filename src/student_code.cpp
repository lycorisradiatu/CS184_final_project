#include "student_code.h"
#include "CGL/matrix3x3.h"
#include "CGL/renderer.h"
#include "CGL/vector2D.h"
#include "CGL/vector3D.h"
#include "halfEdgeMesh.h"
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
    std::vector<Vector2D> ret = std::vector<Vector2D>();
    ret.resize(points.size()-1);
    for (int i = 0; i < points.size() - 1; i++) {
      ret.at(i) = points.at(i)*(1-this->t) + (this->t)*points.at(i+1);
    }
    return ret;
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
    std::vector<Vector3D> ret = std::vector<Vector3D>();
    ret.resize(points.size()-1);
    for (int i = 0; i < points.size() - 1; i++) {
      ret[i] = points.at(i)*(1-t) + (t)*points.at(i+1);
    }
    return ret;
    
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
    std::vector<Vector3D> mask = evaluateStep(points, t);
    while (mask.size() > 1) {
      mask = evaluateStep(mask, t);
    }
    return mask[0];
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
    std::vector<Vector3D> mask = std::vector<Vector3D>();
    mask.resize(this->controlPoints.size());
    for (int row = 0; row < this->controlPoints.size(); row ++) {
      mask[row] = evaluate1D(this->controlPoints[row], u);
    }
    return evaluate1D(mask, v);
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.

    // use the half-edge data structure to iterate through faces (triangles) incident to the vertex
    Vector3D all_area_weighted_normals = Vector3D(0, 0, 0);
    HalfedgeCIter h = this->halfedge();
    do
    {
      if (!h->face()->isBoundary()) {
        VertexCIter v1 = h->vertex();
        VertexCIter v2 = h->next()->vertex();
        VertexCIter v3 = h->next()->next()->vertex();

        // A normal of a face is defined as a vector perpendicular to the surface at a given point
        // The cross product of two vectors along the face would return a third vector perpendicular to the two vectors.
        Vector3D edge1 = v1->position - v2->position;
        Vector3D edge2 = v2->position - v3->position;
        Vector3D normal = cross(edge1, edge2);

        double area = cross(edge1, edge2).norm() / 2;

        all_area_weighted_normals += normal * area;
      }
      h = h->twin()->next();
    }
    while(h != this->halfedge());

    all_area_weighted_normals.normalize();
    return all_area_weighted_normals;
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.

    if (e0->isBoundary()) {
      return e0;
    } else {
      HalfedgeIter h = e0->halfedge();
      HalfedgeIter twin = h->twin();

      VertexIter b = h->vertex();
      VertexIter c = h->twin()->vertex();
      VertexIter a = h->next()->next()->vertex();
      VertexIter d = h->twin()->next()->next()->vertex();

      FaceIter f1 = h->face();
      FaceIter f2 = h->twin()->face();

      HalfedgeIter ac = h->next();
      HalfedgeIter ab = h->next()->next();
      HalfedgeIter bd = h->twin()->next();
      HalfedgeIter cd = h->twin()->next()->next();

      ac->setNeighbors(h, ac->twin(), c, ac->edge(), f1);
      cd->setNeighbors(ac, cd->twin(), d, cd->edge(), f1);
      bd->setNeighbors(twin, bd->twin(), b, bd->edge(), f2);
      ab->setNeighbors(bd, ab->twin(), a, ab->edge(), f2);

      h->setNeighbors(cd, twin, a, e0, f1);
      twin->setNeighbors(ab, h, d, e0, f2);

      e0->halfedge() = h;

      f1->halfedge() = h;
      f2->halfedge() = twin;

      a->halfedge() = h;
      b->halfedge() = bd;
      c->halfedge() = ac;
      d->halfedge() = twin;

      return e0;
    }
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.

    if (e0->isBoundary()) {
      return VertexIter();
    } else {
      HalfedgeIter h = e0->halfedge();
      HalfedgeIter twin = h->twin();

      VertexIter b = h->vertex();
      VertexIter c = h->twin()->vertex();
      VertexIter a = h->next()->next()->vertex();
      VertexIter d = h->twin()->next()->next()->vertex();

      FaceIter f1 = h->face();
      FaceIter f2 = h->twin()->face();

      HalfedgeIter ac = h->next();
      HalfedgeIter ab = h->next()->next();
      HalfedgeIter bd = h->twin()->next();
      HalfedgeIter cd = h->twin()->next()->next();

      // faces go counterclockwise
      // create a new vertex, two new triangles, three new edges
      VertexIter m = newVertex();
      FaceIter f3 = newFace();
      FaceIter f4 = newFace();
      EdgeIter mb = newEdge();
      EdgeIter am = newEdge();
      EdgeIter md = newEdge();

      HalfedgeIter am_half = newHalfedge();
      HalfedgeIter ma_half = newHalfedge();
      HalfedgeIter mb_half = newHalfedge();
      HalfedgeIter bm_half = newHalfedge();
      HalfedgeIter md_half = newHalfedge();
      HalfedgeIter dm_half = newHalfedge();

      m->halfedge() = h;
      m->position = (h->vertex()->position + twin->vertex()->position) / 2;

      ac->setNeighbors(am_half, ac->twin(), c, ac->edge(), f1);
      ab->setNeighbors(bm_half, ab->twin(), a, ab->edge(), f3);
      bd->setNeighbors(dm_half, bd->twin(), b, bd->edge(), f4);
      cd->setNeighbors(twin, cd->twin(), d, cd->edge(), f2);

      h->setNeighbors(ac, twin, m, e0, f1);
      twin->setNeighbors(md_half, h, c, e0, f2);

      am_half->setNeighbors(h, ma_half, a, am, f1);
      ma_half->setNeighbors(ab, am_half, m, am, f3);
      mb_half->setNeighbors(bd, bm_half, m, mb, f4);
      bm_half->setNeighbors(ma_half, mb_half, b, mb, f3);
      md_half->setNeighbors(cd, dm_half, m, md, f2);
      dm_half->setNeighbors(mb_half, md_half, d, md, f4);

      a->halfedge() = am_half;
      b->halfedge() = bd;
      c->halfedge() = twin;
      d->halfedge() = cd;
      
      f1->halfedge() = h;
      f2->halfedge() = twin;
      f3->halfedge() = bm_half;
      f4->halfedge() = mb_half;

      e0->halfedge() = h;
      mb->halfedge() = mb_half;
      am->halfedge() = am_half;
      md->halfedge() = md_half;

      m->isNew = true;
      am->isNew = true;
      md->isNew = true;
      
      return m;
    }
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

    // Step A: Compute the positions of both new and old vertices using the original mesh.

    // (1) compute the position of a newly added vertex
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      HalfedgeIter h = e->halfedge();
      VertexIter a = h->vertex();
      VertexIter b = h->next()->vertex();
      VertexIter c = h->twin()->next()->next()->vertex();
      VertexIter d = h->next()->next()->vertex();
      e->isNew = false;
      e->newPosition = 3.0/8 * a->position + 3.0/8 * b->position + 1.0/8 * c->position + 1.0/8 * d->position;
    }

    // (2) update the position of an existing vertex. 
    for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
      int n = v->degree();
      double u;
      if (n == 3) {
        u = 3.0/16;
      } else {
        u = 3.0/(8 * n);
      }
      Vector3D original_neighbor_position_sum = Vector3D(0, 0, 0);
      HalfedgeIter h = v->halfedge();
      do
      {
        original_neighbor_position_sum += h->next()->vertex()->position;
        h = h->twin()->next();
      }
      while(h != v->halfedge());
      v->newPosition = (1 - n * u) * v->position + u * original_neighbor_position_sum;
      v->isNew = false;
    }

    // Step B: Subdivide the original mesh via edge splits and flips as described.

    // (1) Split every existing edge of the mesh in any order.
    int old_edge_num = mesh.nEdges();
    EdgeIter e = mesh.edgesBegin();
    for (int i = 0; i != old_edge_num; i++) {
      VertexIter v = mesh.splitEdge(e);
      v->newPosition = e->newPosition;
      e++;
    }

    // (2) Flip any new edge that connects an old vertex and a new vertex
    for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
      if (e->isNew && 
      ((e->halfedge()->vertex()->isNew && !e->halfedge()->twin()->vertex()->isNew) ||
      (!e->halfedge()->vertex()->isNew && e->halfedge()->twin()->vertex()->isNew)
      )) {
        mesh.flipEdge(e);
      }
    }
    
    // Step C: Update all vertex positions in the subdivided mesh using the values already computed.
    for (VertexIter v = mesh.verticesBegin(); v!= mesh.verticesEnd(); v++) {
      v->position = v->newPosition;
    }
  }


  VertexIter HalfedgeMesh::collapse (EdgeIter e0) {
    
    HalfedgeIter h0 = e0->halfedge();
      HalfedgeIter h1 = h0->twin();

      VertexIter v0 = h0->vertex();
      VertexIter v1 = h1->vertex();

      Vector3D newPosition = (v0->position + v1->position) / 2.0;

      HalfedgeIter h = v0->halfedge();
      do {
          h->vertex() = v1;
          h = h->twin()->next();
      } while (h != v0->halfedge());
    
      HalfedgeIter h2 = h0->next();
      HalfedgeIter h3 = h2->next();
      HalfedgeIter h4 = h3->twin()->next();
      HalfedgeIter h5 = h4->next();
      HalfedgeIter h6 = h1->next();
      HalfedgeIter h7 = h6->next();
      HalfedgeIter h8 = h6->twin()->next();
      HalfedgeIter h9 = h8->next();
  
      h3->vertex()->halfedge() = h4;
      h7->vertex()->halfedge() = h7;

      h4->face()->halfedge() = h4;
      h8->face()->halfedge() = h8;

      h5->next() = h2;
      h2->next() = h4;
      h9->next() = h7;
      h7->next() = h8;

      
      v1->position = newPosition;
      
      deleteVertex(v0);
      deleteEdge(e0);
      deleteFace(h0->face());
      deleteFace(h1->face());
      deleteEdge(h3->edge());
      deleteEdge(h6->edge());
      deleteHalfedge(h3->twin());
      deleteHalfedge(h3);
      deleteHalfedge(h6->twin());
      deleteHalfedge(h6);
      deleteHalfedge(h0);
      deleteHalfedge(h1);
      return v1;
  }

  VertexIter HalfedgeMesh::shift (VertexIter v0) {
    //part 1 of equation
    Vector3D ci = Vector3D(0,0,0);
    HalfedgeIter begin = v0->halfedge();
  
    do {
      
      begin = begin->next();
      ci += begin->vertex()->position;
      begin = begin->next()->twin();
    } while (begin != v0 ->halfedge());
    ci /= v0->degree();
  


  //part 2 of equation
  
    double lambda = 0.86;
    double I[9] = {1,0,0,0,1,0,0,0,1} ;
    double data[9] = {v0->normal().x * v0->normal().x, v0->normal().x * v0->normal().y, v0->normal().x * v0->normal().z,
                      v0->normal().y * v0->normal().x, v0->normal().y * v0->normal().y, v0->normal().z * v0->normal().z,
                      v0->normal().z * v0->normal().x, v0->normal().y * v0->normal().z, v0->normal().z * v0->normal().z};
    
    
  
    Matrix3x3 identity = Matrix3x3(I);
    Matrix3x3 M = Matrix3x3(data);
   
  
    v0->position = v0->position + lambda*(identity - M) * (ci - v0->position);
    return v0;
  }

  //utilizing the loop-traversing used in hw
  void MeshResampler::remesh ( HalfedgeMesh& mesh) {
    //define Lmax for split edge and Lmin for collapse
    double L = 1.87;
    double L_max = 4/3* L;
    double L_min = 3/5 * L;


    int old_edge_num = mesh.nEdges();
    EdgeIter e = mesh.edgesBegin();
    std::cout << "remshing in progress" << std::endl;
    for (int i = 0; i != old_edge_num; i++) {
      //spliting phase
      std::cout << "spliting phase" << std::endl;
      if (e->halfedge()->edge()->length() > L_max) {
       mesh.splitEdge(e);
      }
      std::cout << "collapse phase" << std::endl;
      //collapse phase
      if (e->halfedge()->edge()->length() < L_min) {
        mesh.collapse(e);
      }
      std::cout << "fliping phase" << std::endl;
      //fliping phase
      if (e->halfedge()->vertex()->degree() > 6) {
        mesh.flipEdge(e);
      }
      std::cout << "shifting phase" << std::endl;
      //shifting and projecting phase
      mesh.shift(e->halfedge()->vertex());
     
      e++;
    }
    std::cout << "remshing done" << std::endl;

    //Spliting phase

    //Collapsing phase

    //Flipping pase



    //shifting and projecting phase





  }

  void MeshResampler::subdivision(HalfedgeMesh& mesh ) {

  }
}
