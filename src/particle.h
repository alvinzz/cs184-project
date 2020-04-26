#ifndef CGL_PARTICLE_H
#define CGL_PARTICLE_H

#include "CGL/CGL.h"
#include "CGL/vector3D.h"
#include "CGL/vector4D.h"
#include "CGL/matrix4x4.h"
#include "CGL/spectrum.h"
#include "halfEdgeMesh.h"

namespace CGL {

  struct Isect {
    bool valid;
    FaceIter face;
    double distance;
    Vector3D barycentric;
  };

  class Particle {
    public:
      double mass;
      double velocity;
      Vector3D direction;
      Vector3D source_position;
      Isect isect;

      Particle(Vector3D& source_position, Vector3D& direction, double mass, double velocity) {
        this->source_position = source_position;
        this->direction = direction;
        this->mass = mass;
        this->velocity = velocity;
        this->isect = Isect;
        this->isect.valid = false;
      }

      bool intersect(FaceIter& face);
      void dentFace( Particle& p );
  };

}  // namespace CGL

#endif  // CGL_RAY_H