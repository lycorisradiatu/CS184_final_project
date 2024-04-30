#include "student_code.h"
#include "CGL/renderer.h"
#include "CGL/vector2D.h"
#include "CGL/vector3D.h"
#include "halfEdgeMesh.h"
#include "mutablePriorityQueue.h"
#include <cmath>

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
          e->newPosition = 3.0 / 8 * a->position + 3.0 / 8 * b->position + 1.0 / 8 * c->position + 1.0 / 8 * d->position;
      }

      // (2) update the position of an existing vertex. 
      for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
          int n = v->degree();
          double u;
          if (n == 3) {
              u = 3.0 / 16;
          }
          else {
              u = 3.0 / (8 * n);
          }
          Vector3D original_neighbor_position_sum = Vector3D(0, 0, 0);
          HalfedgeIter h = v->halfedge();
          do
          {
              original_neighbor_position_sum += h->next()->vertex()->position;
              h = h->twin()->next();
          } while (h != v->halfedge());
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
      for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
          v->position = v->newPosition;
      }
  }
  
  double MeshResampler::get_s_j(int j, int K) {
    if (K == 3) {
      if (j == 0) {
        return 5.0 / 12;
      } else {
        return - 1.0 / 12;
      }
    }
    else if (K == 4) {
      if (j == 0) {
        return 3.0 / 8;
      } else if (j == 2) {
        return -1.0 / 8;
      } else {
        return 0;
      }
    }
    else {
      return (0.25 + cos((2 * M_PI * j) / K) + 0.5 * cos((4 * M_PI * j) / K)) / K;
    }
  }

  


  VertexIter HalfedgeMesh::splitFace(FaceIter face)
  {

      if (face->isBoundary()) {
          return VertexIter();
      }
      else {
          HalfedgeIter h1 = face->halfedge();
          VertexIter a = h1->vertex();
          VertexIter b = h1->next()->vertex();
          VertexIter c = h1->next()->next()->vertex();

          HalfedgeIter h2 = h1->next();
          HalfedgeIter h3 = h2->next();
          HalfedgeIter h4 = newHalfedge();
          HalfedgeIter h5 = newHalfedge();
          HalfedgeIter h6 = newHalfedge();
          HalfedgeIter h7 = newHalfedge();
          HalfedgeIter h8 = newHalfedge();
          HalfedgeIter h9 = newHalfedge();

          VertexIter m = newVertex();
          EdgeIter e1 = newEdge();
          EdgeIter e2 = newEdge();
          EdgeIter e3 = newEdge();

          FaceIter f1 = face;
          face->is_new = true;
          FaceIter f2 = newFace();
          FaceIter f3 = newFace();

          // verteces
          m->halfedge() = h4;
          m->isNew = true;
          m->position = face->newPosition;

          // edges
          e1->halfedge() = h4;
          e2->halfedge() = h6;
          e3->halfedge() = h9;
          e1->isNew = true;
          e2->isNew = true;
          e3->isNew = true;

          // faces
          f1->halfedge() = h1;
          f2->halfedge() = h2;
          f3->halfedge() = h3;

          // halfedges
          h1->setNeighbors(h5, h1->twin(), a, h1->edge(), f1);
          h2->setNeighbors(h7, h2->twin(), b, h2->edge(), f2);
          h3->setNeighbors(h8, h3->twin(), c, h3->edge(), f3);
          h4->setNeighbors(h1, h8, m, e1, f1);
          h5->setNeighbors(h4, h6, b, e2, f1);
          h6->setNeighbors(h2, h5, m, e2, f2);
          h7->setNeighbors(h6, h9, c, e3, f2);
          h8->setNeighbors(h9, h4, a, e1, f3);
          h9->setNeighbors(h3, h7, m, e3, f3);
          return m;

      }
  }

    void MeshResampler::upsample_butterfly_scheme( HalfedgeMesh& mesh ) {
    // (1) compute the position of a newly added vertex
      for (FaceIter f = mesh.facesBegin(); f != mesh.facesEnd(); f++) {
          HalfedgeIter h = f->halfedge();
          VertexIter a = h->vertex();
          VertexIter b = h->next()->vertex();
          VertexIter c = h->next()->next()->vertex();
          f->newPosition = (a->position + b->position + c->position) / 3.0;
          f->is_new = false;
      }

      // (2) update the position of an existing vertex. 
      for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
          int n = v->degree();
          double a_n = (4 - 2 * cos((2.0 * M_PI / n))) / 9;
          Vector3D original_neighbor_position_sum = Vector3D(0, 0, 0);
          HalfedgeIter h = v->halfedge();
          do
          {
              original_neighbor_position_sum += h->next()->vertex()->position;
              h = h->twin()->next();
          } while (h != v->halfedge());
          v->newPosition = (1 - a_n) * v->position + (a_n/n) * original_neighbor_position_sum;
          v->isNew = false;
      }

      // Step B: Subdivide the original mesh.

      // (1) Create new vertex and edges for every face
      for (FaceIter f = mesh.facesBegin(); f != mesh.facesEnd(); f++) {
          if (!f->is_new) {
              mesh.splitFace(f);
          }
      }

      // (2) Flip every original edge that connects two old vertices
     //4. Flip any new edge that connects an old and new vertex.
      for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
          if ((!e->isNew)) {
              VertexIter v1 = e->halfedge()->vertex();
              VertexIter v2 = e->halfedge()->twin()->vertex();

              if (!v1->isNew && !v2->isNew) {
                  mesh.flipEdge(e);
              }
          }
      }

      // Step C: Update all original vertex positions in the subdivided mesh using the values already computed.
      for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
          if (!v->isNew) {
              v->position = v->newPosition;
          }
      }
  }
}
