#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

  // TODO (Part 2.2):
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.

  double t_x0 = (min.x - r.o.x) / r.d.x;
  double t_x1 = (max.x - r.o.x) / r.d.x;
  double t_y0 = (min.y - r.o.y) / r.d.y;
  double t_y1 = (max.y - r.o.y) / r.d.y;
  double t_z0 = (min.z - r.o.z) / r.d.z;
  double t_z1 = (max.z - r.o.z) / r.d.z;

  double t_min_x = std::min(t_x0, t_x1);
  double t_max_x = std::max(t_x0, t_x1);
  double t_min_y = std::min(t_y0, t_y1);
  double t_max_y = std::max(t_y0, t_y1);
  double t_min_z = std::min(t_z0, t_z1);
  double t_max_z = std::max(t_z0, t_z1);

  double t_min = std::max(std::max(t_min_x, t_min_y), t_min_z);
  double t_max = std::min(std::min(t_max_x, t_max_y), t_max_z);

  if (t_min > t_max) {
    return false;
  }

  if (t_max < t0) {
    return false;
  }

  if (t_min > t1) {
    return false;
  }

  return true;
}

void BBox::draw(Color c, float alpha) const {

  glColor4f(c.r, c.g, c.b, alpha);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL
