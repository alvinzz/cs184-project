#include "halfEdgeMesh.h"

namespace CGL {
  bool Particle::intersect(FaceIter& face) {
    Vector3D p1 = face->halfedge()->vertex()->position;
    Vector3D p2 = face->halfedge()->next()->vertex()->position;
    Vector3D p3 = face->halfedge()->next()->next()->vertex()->position;

    Vector3D e1 = p2 - p1;
    Vector3D e2 = p3 - p1;
    Vector3D s = source_position - p1;
    Vector3D s1 = cross(direction, e2);
    Vector3D s2 = cross(s, e1);
    double t = dot(s2, e2) / dot(s1, e1);
    double b1 = dot(s1, s) / dot(s1, e1);
    double b2 = dot(s2, direction) / dot(s1, e1);
    double b0 = 1. - b1 - b2;

    if ((b1 >= 0) && (b2 >= 0) && (b0 >= 0) && 
      (b1 <= 1) && (b2 <= 1) && (b0 <= 1) &&
      (t > 0) && (t < isect.distance)) {
      isect.valid = true;
      isect.face = face;
      isect.distance = t;
      return true;
    }
    return false;
  }

}