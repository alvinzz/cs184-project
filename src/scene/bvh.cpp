#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

double sah(BBox &bbox, int n_primitives) {
  return bbox.surface_area() * (double)n_primitives;
}

bool x_comp(Primitive *i, Primitive *j) {
  return i->get_bbox().centroid().x < j->get_bbox().centroid().x;
}
bool y_comp(Primitive *i, Primitive *j) {
  return i->get_bbox().centroid().y < j->get_bbox().centroid().y;
}
bool z_comp(Primitive *i, Primitive *j) {
  return i->get_bbox().centroid().z < j->get_bbox().centroid().z;
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {

  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.

  BBox bbox;

  vector<Primitive*> primitives;
  for (auto p = start; p != end; p++) {
    BBox bb = (*p)->get_bbox();
    primitives.push_back(*p);
    bbox.expand(bb);
  }

  BVHNode *node = new BVHNode(bbox);

  if (primitives.size() <= max_leaf_size) {
    node->start = start;
    node->end = end;
    node->l = NULL;
    node->r = NULL;
    return node;
  }

  std::vector<double> x_costs;
  std::vector<double> y_costs;
  std::vector<double> z_costs;

  int N = max(2, min(16, (int)primitives.size() / (int)max_leaf_size));
  int group_size = (int)primitives.size() / N;

  std::sort(primitives.begin(), primitives.end(), x_comp);
  for (int split_idx = group_size; split_idx < primitives.size(); split_idx += group_size) {
    BBox left_bb;
    BBox right_bb;
    for (int i = 0; i < split_idx; i++) {
      left_bb.expand(primitives[i]->get_bbox());
    }
    for (int i = split_idx; i < primitives.size(); i++) {
      right_bb.expand(primitives[i]->get_bbox());
    }
    x_costs.push_back(sah(left_bb, split_idx) + sah(right_bb, primitives.size() - split_idx));
  }

  std::sort(primitives.begin(), primitives.end(), y_comp);
  for (int split_idx = group_size; split_idx < primitives.size(); split_idx += group_size) {
    BBox left_bb;
    BBox right_bb;
    for (int i = 0; i < split_idx; i++) {
      left_bb.expand(primitives[i]->get_bbox());
    }
    for (int i = split_idx; i < primitives.size(); i++) {
      right_bb.expand(primitives[i]->get_bbox());
    }
    y_costs.push_back(sah(left_bb, split_idx) + sah(right_bb, primitives.size() - split_idx));
  }

  std::sort(primitives.begin(), primitives.end(), z_comp);
  for (int split_idx = group_size; split_idx < primitives.size(); split_idx += group_size) {
    BBox left_bb;
    BBox right_bb;
    for (int i = 0; i < split_idx; i++) {
      left_bb.expand(primitives[i]->get_bbox());
    }
    for (int i = split_idx; i < primitives.size(); i++) {
      right_bb.expand(primitives[i]->get_bbox());
    }
    z_costs.push_back(sah(left_bb, split_idx) + sah(right_bb, primitives.size() - split_idx));
  }

  double x_min = *min_element(x_costs.begin(), x_costs.end());
  double y_min = *min_element(y_costs.begin(), y_costs.end());
  double z_min = *min_element(z_costs.begin(), z_costs.end());

  int split_idx;
  if ((x_min < y_min) && (x_min < z_min)) {
    std::sort(primitives.begin(), primitives.end(), x_comp);
    int min_idx = min_element(x_costs.begin(), x_costs.end()) - x_costs.begin();
    split_idx = group_size * (1+min_idx);
  } else if (y_min < z_min) {
    std::sort(primitives.begin(), primitives.end(), y_comp);
    int min_idx = min_element(y_costs.begin(), y_costs.end()) - y_costs.begin();
    split_idx = group_size * (1+min_idx);
  } else {
    std::sort(primitives.begin(), primitives.end(), z_comp);
    int min_idx = min_element(z_costs.begin(), z_costs.end()) - z_costs.begin();
    split_idx = group_size * (1+min_idx);
  }

  vector<Primitive*>::iterator itr = start;
  for (int i = 0; i < split_idx; i++) {
    *itr = primitives[i];
    advance(itr, 1);
  }
  for (int i = split_idx; i < primitives.size(); i++) {
    *itr = primitives[i];
    advance(itr, 1);
  }
  node->l = construct_bvh(start, start + split_idx, max_leaf_size);
  node->r = construct_bvh(start + split_idx, end, max_leaf_size);

  return node;
  
}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.

  double t0 = ray.min_t;
  double t1 = ray.max_t;
  if (!(node->bb.intersect(ray, t0, t1))) {
    return false;
  }

  if ((node->l)!=NULL) {
    return (has_intersection(ray, node->l) || has_intersection(ray, node->r));
  }

  for (auto p = node->start; p != node->end; p++) {
    if ((*p)->has_intersection(ray)) {
      return true;
    }
  }
  return false;
}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.

  double t0 = ray.min_t;
  double t1 = ray.max_t;
  if (!(node->bb.intersect(ray, t0, t1))) {
    return false;
  }

  if ((node->l)==NULL) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->intersect(ray, i);
      total_isects++;
    }
    if ((i->primitive)==NULL) {
      return false;
    }
    return true;
  }

  intersect(ray, i, node->l);
  intersect(ray, i, node->r);
  if ((i->primitive)==NULL) {
    return false;
  }
  return true;
}

} // namespace SceneObjects
} // namespace CGL
