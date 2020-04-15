#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.

  return true;
}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
  double b = 2 * dot(r.o - o, r.d);
  double c = dot(r.o - o, r.o - o) - r2;
  double sqrt_term = b*b - 4*c;
  if (sqrt_term < 0) {
    return false;
  }
  double smaller_t = 0.5 * (-b - sqrt(sqrt_term));
  double larger_t = 0.5 * (-b + sqrt(sqrt_term));
  if ((smaller_t >= r.min_t) && (smaller_t <= r.max_t)) {
    r.max_t = smaller_t;
    return true;
  }
  if ((larger_t >= r.min_t) && (larger_t <= r.max_t)) {
    r.max_t = larger_t;
    return true;
  }
  return false;
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.

  double b = 2 * dot(r.o - o, r.d);
  double c = dot(r.o - o, r.o - o) - r2;
  double sqrt_term = b*b - 4*c;
  if (sqrt_term < 0) {
    return false;
  }
  double smaller_t = 0.5 * (-b - sqrt(sqrt_term));
  double larger_t = 0.5 * (-b + sqrt(sqrt_term));
  if ((smaller_t >= r.min_t) && (smaller_t <= r.max_t)) {
    r.max_t = smaller_t;
    i->t = smaller_t;
    i->n = (r.o + smaller_t*r.d - o).unit();
    i->primitive = this;
    i->bsdf = get_bsdf();
    return true;
  }
  if ((larger_t >= r.min_t) && (larger_t <= r.max_t)) {
    r.max_t = larger_t;
    i->t = larger_t;
    i->n = (r.o + larger_t*r.d - o).unit();
    i->primitive = this;
    i->bsdf = get_bsdf();
    return true;
  }
  return false;
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
