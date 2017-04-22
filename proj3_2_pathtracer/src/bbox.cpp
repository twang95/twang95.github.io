#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {

  // Part 2, Task 2:
  // Implement ray - bounding box intersection test
  // If the ray intersected the bouding box within the range given by
  // t0, t1, update t0 and t1 with the new intersection times.

  // O + td = x_min
  // t = (x_min - O) / d

  if ( (r.o.x >= this->min.x) && (r.o.y >= this->min.y) && (r.o.z >= this->min.z)
        && (r.o.x <= this->max.x) && (r.o.y <= this->max.y) && (r.o.z <= this->max.z)) {
    return true;
  }

  double x_min;
  double x_max;

  double x1 = (this->min.x - r.o.x) / r.d.x;
  double x2 = (this->max.x - r.o.x) / r.d.x;

  x_min = std::min(x1, x2);
  x_max = std::max(x1, x2);

  double y_min;
  double y_max;

  double y1 = (this->min.y - r.o.y) / r.d.y;
  double y2 = (this->max.y - r.o.y) / r.d.y;

  y_min = std::min(y1, y2);
  y_max = std::max(y1, y2);

  double z_min;
  double z_max;

  double z1 = (this->min.z - r.o.z) / r.d.z;
  double z2 = (this->max.z - r.o.z) / r.d.z;

  z_min = std::min(z1, z2);
  z_max = std::max(z1, z2);

  double t_mins;
  double t_maxes;

  t_mins = std::max(t0, std::max(x_min, std::max(y_min, z_min)));
  t_maxes = std::min(t1, std::min(x_max, std::min(y_max, z_max)));

  if (t_mins <= t_maxes) {
    t0 = t_mins;
    t1 = t_maxes;
    return true;
  }

  return false;
}

void BBox::draw(Color c) const {

  glColor4f(c.r, c.g, c.b, c.a);

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
