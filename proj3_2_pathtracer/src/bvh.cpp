#include "bvh.h"

#include "CGL/CGL.h"
#include "static_scene/triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL { namespace StaticScene {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  root = construct_bvh(_primitives, max_leaf_size);

}

BVHAccel::~BVHAccel() {
  if (root) delete root;
}

BBox BVHAccel::get_bbox() const {
  return root->bb;
}

void BVHAccel::draw(BVHNode *node, const Color& c) const {
  if (node->isLeaf()) {
    for (Primitive *p : *(node->prims))
      p->draw(c);
  } else {
    draw(node->l, c);
    draw(node->r, c);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color& c) const {
  if (node->isLeaf()) {
    for (Primitive *p : *(node->prims))
      p->drawOutline(c);
  } else {
    drawOutline(node->l, c);
    drawOutline(node->r, c);
  }
}

BVHNode *BVHAccel::construct_bvh(const std::vector<Primitive*>& prims, size_t max_leaf_size) {
  
  // Part 2, Task 1:
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.


  BBox centroid_box, bbox;

  for (Primitive *p : prims) {
    BBox bb = p->get_bbox();
    bbox.expand(bb);
    Vector3D c = bb.centroid();
    centroid_box.expand(c);
  }

  BVHNode *node = new BVHNode(bbox);

  if (prims.size() > max_leaf_size) {
    std::vector<Primitive *> leftSide;
    std::vector<Primitive *> rightSide;

    // divide it in half by the x coordinate, y and z coordinates will be the same
    // splitBy will be 0 for x, 1 for y, and 2 for z
    int splitBy;
    Vector3D split_point;
    if (centroid_box.extent.x >= centroid_box.extent.y && centroid_box.extent.x >= centroid_box.extent.z) {
      splitBy = 0;
       split_point = Vector3D(centroid_box.max.x - (centroid_box.extent.x / 2), centroid_box.centroid().y, centroid_box.centroid().z);
    } else if (centroid_box.extent.y >= centroid_box.extent.z && centroid_box.extent.y >= centroid_box.extent.x) {
      splitBy = 1;
       split_point = Vector3D(centroid_box.centroid().x, centroid_box.max.y - (centroid_box.extent.y / 2), centroid_box.centroid().z);
    } else if (centroid_box.extent.z >= centroid_box.extent.x && centroid_box.extent.z >= centroid_box.extent.y) {
      splitBy = 2;
       split_point = Vector3D(centroid_box.centroid().x, centroid_box.centroid().y, centroid_box.max.z - (centroid_box.extent.z / 2));
    } 
    
    for (Primitive *p: prims) {
      Vector3D prim_centroid = p->get_bbox().centroid();
      if (splitBy == 0) {
        if (prim_centroid.x >= split_point.x) {
          rightSide.push_back(p);
        } else {
          leftSide.push_back(p);
        }
      } else if (splitBy == 1) {
        if (prim_centroid.y >= split_point.y) {
          rightSide.push_back(p);
        } else {
          leftSide.push_back(p);
        }
      } else if (splitBy == 2) {
        if (prim_centroid.z >= split_point.z) {
          rightSide.push_back(p);
        } else {
          leftSide.push_back(p);
        }
      }
    }
    if (leftSide.size() != 0) {
      node->l = construct_bvh(leftSide, max_leaf_size);
    } 
    if (rightSide.size() != 0) {
      node->r = construct_bvh(rightSide, max_leaf_size);
    }
  }
  node->prims = new vector<Primitive *>(prims);
  return node;

}


bool BVHAccel::intersect(const Ray& ray, BVHNode *node) const {
  // Part 2, task 3: replace this.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.

  double t0 = ray.min_t;
  double t1 = ray.max_t;
  if (node->bb.intersect(ray, t0, t1)) {
    if (t0 > ray.max_t || t1 < ray.min_t) {
      return false;
    }
    if (node->isLeaf()) {
      for (Primitive *p: *(node->prims)) {
        if (p->intersect(ray)) {
            return true;
        }
      }
      return false;
    } else {
      return intersect(ray, node->l) || intersect(ray, node->r);
    }
  }
  
  return false;
}

bool BVHAccel::intersect(const Ray& ray, Intersection* i, BVHNode *node) const {
  // Part 2, task 3: replace this

  // bool hit = false;
  // for (Primitive *p: *(root->prims)) {
  //   total_isects++;
  //   if (p->intersect(ray, i))
  //     hit = true;
  // }
  // return hit;

  double t0 = ray.min_t;
  double t1 = ray.max_t;
  if (node->bb.intersect(ray, t0, t1)) {
    if (t0 > ray.max_t || t1 < ray.min_t) {
      return false;
    }
    if (node->isLeaf()) {
      double closest_t = HUGE_VAL;
      bool intersected = false;
      Intersection temp_isect;
      for (Primitive *p: *(node->prims)) {
        if (p->intersect(ray, &temp_isect)) {
          if (temp_isect.t < closest_t) {
            closest_t = temp_isect.t;
            i->t = temp_isect.t;
            i->bsdf = temp_isect.bsdf;
            i->n = temp_isect.n;
            i->primitive = temp_isect.primitive;
          }
          intersected = true;
        }
      }
      return intersected;
    } else {
      Intersection left_isect;
      Intersection right_isect;

      bool hit1, hit2 = false;

      if (node->l != NULL) {
        hit1 = intersect(ray, &left_isect, node->l);
      }
      if (node->r != NULL) {
        hit2 = intersect(ray, &right_isect, node->r);
      }

      if (hit1 && hit2) {
        if (left_isect.t < right_isect.t) {
          i->t = left_isect.t;
          i->bsdf = left_isect.bsdf;
          i->n = left_isect.n;
          i->primitive = left_isect.primitive;
        } else {
          i->t = right_isect.t;
          i->bsdf = right_isect.bsdf;
          i->n = right_isect.n;
          i->primitive = right_isect.primitive;
        }
        return true;
      } else if (hit1) {
        i->t = left_isect.t;
        i->bsdf = left_isect.bsdf;
        i->n = left_isect.n;
        i->primitive = left_isect.primitive;
        return true;
      } else if (hit2) {
        i->t = right_isect.t;
        i->bsdf = right_isect.bsdf;
        i->n = right_isect.n;
        i->primitive = right_isect.primitive;
        return true;
      }
      return false;
    }
  }
  return false;
}

}  // namespace StaticScene
}  // namespace CGL
