#include "sphere.h"

#include <cmath>

#include  "../bsdf.h"
#include "../misc/sphere_drawing.h"

namespace CGL { namespace StaticScene {

bool Sphere::test(const Ray& r, double& t1, double& t2) const {

  // Part 1, Task 4:
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.

  return false;
}

bool Sphere::intersect(const Ray& r) const {

  // Part 1, Task 4:
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
  
  double a = dot(r.d, r.d);
  double b = dot(2.0 * (r.o - this->o), r.d);
  double c = dot(r.o - this->o, r.o - this->o) - this->r2;

  double rootTerm = pow(b, 2.0) - 4.0 * a * c;

  if (rootTerm < 0.0) {
    return false;
  }
  double t1 = (-1.0 * b + sqrt(rootTerm)) / (2.0 * a);
  double t2 = (-1.0 * b - sqrt(rootTerm)) / (2.0 * a);

  double min_root = min(t1, t2);
  double max_root = max(t1, t2);

  if ((min_root < r.min_t || min_root > r.max_t) && (max_root < r.min_t || max_root > r.max_t)) {
    return false;
  } else if (min_root >= r.min_t && min_root <= r.max_t) {
    r.max_t = min_root;
    return true;
  } else {
    r.max_t = max_root;
    return true;
  }

  // if (min_root >= r.min_t && min_root <= r.max_t) {
  //   r.max_t = min_root;
  //   return true;
  // } else if (max_root >= r.min_t && max_root <= r.max_t) {
  //   r.max_t = max_root;
  //   return true;
  // } else {
  //   return false;
  // }


}

bool Sphere::intersect(const Ray& r, Intersection *i) const {

  // Part 1, Task 4:
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.

  double a = dot(r.d, r.d);
  double b = dot(2.0 * (r.o - this->o), r.d);
  double c = dot(r.o - this->o, r.o - this->o) - this->r2;

  double rootTerm = pow(b, 2.0) - 4.0 * a * c;

  if (rootTerm < 0.0) {
    return false;
  }
  double t1 = (-1.0 * b + sqrt(rootTerm)) / (2.0 * a);
  double t2 = (-1.0 * b - sqrt(rootTerm)) / (2.0 * a);



  double min_root = min(t1, t2);
  double max_root = max(t1, t2);

  if ((min_root < r.min_t || min_root > r.max_t) && (max_root < r.min_t || max_root > r.max_t)) {
    return false;
  } else if (min_root >= r.min_t && min_root <= r.max_t) {
    Vector3D hitpoint = r.o + min_root * r.d;
    Vector3D normal = hitpoint - this->o;

    r.max_t = min_root;
    i->t = min_root;
    i->primitive = this;
    i->bsdf = get_bsdf();
    i->n = normal.unit();
    return true;
  } else {
    Vector3D hitpoint = r.o + max_root * r.d;
    Vector3D normal = hitpoint - this->o;

    r.max_t = max_root;
    i->t = max_root;
    i->primitive = this;
    i->bsdf = get_bsdf();
    i->n = normal.unit();
    return true;
  }
}

void Sphere::draw(const Color& c) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color& c) const {
    //Misc::draw_sphere_opengl(o, r, c);
}


} // namespace StaticScene
} // namespace CGL
