#include "halfEdgeMesh.h"
#define MAX_DISPLACEMENT 0.001
#define SHEAR_HARDNESS_BASE 1
#define FRONT_HARDNESS_BASE 25
#define EPS_D 0.00000000001
#define PI 3.14159265359
#define CREASE_THRESH PI / 2.

namespace CGL {
  void BBox::draw(Color c, float alpha) {

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

  bool x_comp(FaceIter& i, FaceIter& j) {
    BBox b_i = BBox(i);
    BBox b_j = BBox(j);
    return b_i.centroid().x < b_j.centroid().x;
  }
  bool y_comp(FaceIter& i, FaceIter& j) {
    BBox b_i = BBox(i);
    BBox b_j = BBox(j);
    return b_i.centroid().y < b_j.centroid().y;
  }
  bool z_comp(FaceIter& i, FaceIter& j) {
    BBox b_i = BBox(i);
    BBox b_j = BBox(j);
    return b_i.centroid().z < b_j.centroid().z;
  }
  double sah(BBox &bbox, int n_primitives) {
    return bbox.surface_area() * (double)n_primitives;
  }

  void BBox::expand(FaceIter& face) {
    Vector3D p1 = face->halfedge()->vertex()->position;
    Vector3D p2 = face->halfedge()->next()->vertex()->position;
    Vector3D p3 = face->halfedge()->next()->next()->vertex()->position;
    min.x = std::min(std::min(std::min(min.x, p1.x), p2.x), p3.x);
    min.y = std::min(std::min(std::min(min.y, p1.y), p2.y), p3.y);
    min.z = std::min(std::min(std::min(min.z, p1.z), p2.z), p3.z);
    max.x = std::max(std::max(std::max(max.x, p1.x), p2.x), p3.x);
    max.y = std::max(std::max(std::max(max.y, p1.y), p2.y), p3.y);
    max.z = std::max(std::max(std::max(max.z, p1.z), p2.z), p3.z);
  }

  BVHNode* BVHTree::construct_bvh(vector<FaceIter>::iterator start, vector<FaceIter>::iterator end) {
    BBox bbox = BBox();

    vector<FaceIter> faces = vector<FaceIter>();
    for (vector<FaceIter>::iterator f = start; f != end; f++) {
      bbox.expand(*f);
      faces.push_back(*f);
    }

    BVHNode *node = new BVHNode(bbox);

    if (faces.size() <= max_leaf_size) {
      node->start = start;
      node->end = end;
      node->l = NULL;
      node->r = NULL;
      for (vector<FaceIter>::iterator f = start; f != end; f++) {
        (*f)->bvh_node = node;
      }
      return node;
    }

    std::vector<double> x_costs;
    std::vector<double> y_costs;
    std::vector<double> z_costs;

    int N = max(2, min(16, (int)faces.size() / (int)max_leaf_size));
    int group_size = (int)faces.size() / N;

    std::sort(faces.begin(), faces.end(), x_comp);
    for (int split_idx = group_size; split_idx < faces.size(); split_idx += group_size) {
      BBox left_bb;
      BBox right_bb;
      for (int i = 0; i < split_idx; i++) {
        left_bb.expand(faces[i]);
      }
      for (int i = split_idx; i < faces.size(); i++) {
        right_bb.expand(faces[i]);
      }
      x_costs.push_back(sah(left_bb, split_idx) + sah(right_bb, faces.size() - split_idx));
    }

    std::sort(faces.begin(), faces.end(), y_comp);
    for (int split_idx = group_size; split_idx < faces.size(); split_idx += group_size) {
      BBox left_bb;
      BBox right_bb;
      for (int i = 0; i < split_idx; i++) {
        left_bb.expand(faces[i]);
      }
      for (int i = split_idx; i < faces.size(); i++) {
        right_bb.expand(faces[i]);
      }
      y_costs.push_back(sah(left_bb, split_idx) + sah(right_bb, faces.size() - split_idx));
    }

    std::sort(faces.begin(), faces.end(), z_comp);
    for (int split_idx = group_size; split_idx < faces.size(); split_idx += group_size) {
      BBox left_bb;
      BBox right_bb;
      for (int i = 0; i < split_idx; i++) {
        left_bb.expand(faces[i]);
      }
      for (int i = split_idx; i < faces.size(); i++) {
        right_bb.expand(faces[i]);
      }
      z_costs.push_back(sah(left_bb, split_idx) + sah(right_bb, faces.size() - split_idx));
    }

    double x_min = *min_element(x_costs.begin(), x_costs.end());
    double y_min = *min_element(y_costs.begin(), y_costs.end());
    double z_min = *min_element(z_costs.begin(), z_costs.end());

    int split_idx;
    if ((x_min < y_min) && (x_min < z_min)) {
      std::sort(faces.begin(), faces.end(), x_comp);
      int min_idx = min_element(x_costs.begin(), x_costs.end()) - x_costs.begin();
      split_idx = group_size * (1+min_idx);
    } else if (y_min < z_min) {
      std::sort(faces.begin(), faces.end(), y_comp);
      int min_idx = min_element(y_costs.begin(), y_costs.end()) - y_costs.begin();
      split_idx = group_size * (1+min_idx);
    } else {
      std::sort(faces.begin(), faces.end(), z_comp);
      int min_idx = min_element(z_costs.begin(), z_costs.end()) - z_costs.begin();
      split_idx = group_size * (1+min_idx);
    }

    vector<FaceIter>::iterator itr = start;
    for (int i = 0; i < split_idx; i++) {
      *itr = faces[i];
      advance(itr, 1);
    }
    for (int i = split_idx; i < faces.size(); i++) {
      *itr = faces[i];
      advance(itr, 1);
    }
    node->l = construct_bvh(start, start + split_idx);
    node->r = construct_bvh(start + split_idx, end);

    node->l->p = node;
    node->r->p = node;

    return node;
  }

  bool Particle::intersect(BVHNode* n) {
    if (!intersect(n->bb)) {
      return false;
    }

    if (n->isLeaf()) {
      for (vector<FaceIter>::iterator f = n->start; f != n->end; f++) {
        intersect(*f);
      }
      return isect.valid;
    }

    intersect(n->l);
    intersect(n->r);
    return isect.valid;
  }

  bool Particle::intersect(BBox& bbox) {
    double t_x0 = (bbox.min.x - source_position.x) / direction.x;
    double t_x1 = (bbox.max.x - source_position.x) / direction.x;
    double t_y0 = (bbox.min.y - source_position.y) / direction.y;
    double t_y1 = (bbox.max.y - source_position.y) / direction.y;
    double t_z0 = (bbox.min.z - source_position.z) / direction.z;
    double t_z1 = (bbox.max.z - source_position.z) / direction.z;

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

    if (t_max < 0) {
      return false;
    }

    if (t_min > isect.distance) {
      return false;
    }

    return true;
  }

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
      isect.barycentric = Vector3D(b0, b1, b2);
      isect.position = b0*p1 + b1*p2 + b2*p3;
      return true;
    }
    return false;
  }

  void BVHNode::update_bvh(Vector3D pos) {
    bool updated_bb = false;
    if (
        (bb.min.x != std::min(bb.min.x, pos.x))
        || (bb.min.y != std::min(bb.min.y, pos.y))
        || (bb.min.z != std::min(bb.min.z, pos.z))
        || (bb.max.x != std::max(bb.max.x, pos.x))
        || (bb.max.y != std::max(bb.max.y, pos.y))
        || (bb.max.z != std::max(bb.max.z, pos.z))
    ) {
      updated_bb = true;
    }
    bb.min.x = std::min(bb.min.x, pos.x);
    bb.min.y = std::min(bb.min.y, pos.y);
    bb.min.z = std::min(bb.min.z, pos.z);
    bb.max.x = std::max(bb.max.x, pos.x);
    bb.max.y = std::max(bb.max.y, pos.y);
    bb.max.z = std::max(bb.max.z, pos.z);
    if ((updated_bb) && (p)) {
      p->update_bvh(pos);
    }
  }

  void correct_crease(FaceIter& f1, FaceIter& f2, VertexIter& v1, VertexIter& v2) {
    // face normals always point out/into the mesh due to CCW winding constraint 
    Vector3D n1 = f1->normal();
    Vector3D n2 = f2->normal();

    double angle_init = acos(dot(n1, n2));
    if (angle_init > CREASE_THRESH) {
      v1->position += MAX_DISPLACEMENT * (n1 + n2).unit();
      v2->position += MAX_DISPLACEMENT * (n1 + n2).unit();
      double new_angle = acos(dot(f1->normal(), f2->normal()));
      if (new_angle < angle_init) {
        int ctr = 0;
        while ((new_angle > CREASE_THRESH) && (ctr < 10)) {
          n1 = f1->normal();
          n2 = f2->normal();
          v1->position += MAX_DISPLACEMENT * (n1 + n2).unit();
          v2->position += MAX_DISPLACEMENT * (n1 + n2).unit();
          new_angle = acos(dot(f1->normal(), f2->normal()));
          ctr++;
        }
      } else {
        v1->position -= MAX_DISPLACEMENT * (n1 + n2).unit();
        v2->position -= MAX_DISPLACEMENT * (n1 + n2).unit();
        new_angle = acos(dot(f1->normal(), f2->normal()));
        int ctr = 0;
        while ((new_angle > CREASE_THRESH) && (ctr < 10)) {
          n1 = f1->normal();
          n2 = f2->normal();
          v1->position -= MAX_DISPLACEMENT * (n1 + n2).unit();
          v2->position -= MAX_DISPLACEMENT * (n1 + n2).unit();
          new_angle = acos(dot(f1->normal(), f2->normal()));
          ctr++;
        }
      }
    }
  }

  void Particle::dentFace() {
      VertexIter v1 = isect.face->halfedge()->vertex();
      VertexIter v2 = isect.face->halfedge()->next()->vertex();
      VertexIter v3 = isect.face->halfedge()->next()->next()->vertex();

      double area = isect.face->area();
      Vector3D energy = 0.5 * mass * velocity * velocity 
        * isect.barycentric / area; 

      Vector3D n = isect.face->normal();
      if (dot(direction, n) < 0) {
        n = -n;
      }
      // two displacements:
      //  Erosion from shearing off bits and pieces (move normal to face)
      //  Erosion from head-on collisions denting the rock (move in direction of particle)
      Vector3D v1_update = energy.x
        * (sin(acos(dot(direction, n))) / 40000. / SHEAR_HARDNESS_BASE * n
          + dot(direction, n) / 40000. / FRONT_HARDNESS_BASE * direction);
      Vector3D v2_update = energy.y
        * (sin(acos(dot(direction, n))) / 40000. / SHEAR_HARDNESS_BASE * n
          + dot(direction, n) / 40000. / FRONT_HARDNESS_BASE * direction);
      Vector3D v3_update = energy.z
        * (sin(acos(dot(direction, n))) / 40000. / SHEAR_HARDNESS_BASE * n
          + dot(direction, n) / 40000. / FRONT_HARDNESS_BASE * direction);

      if (v1_update.norm() > MAX_DISPLACEMENT) {
        v1_update /= (v1_update.norm() / MAX_DISPLACEMENT);
      }
      if (v2_update.norm() > MAX_DISPLACEMENT) {
        v2_update /= (v2_update.norm() / MAX_DISPLACEMENT);
      }
      if (v3_update.norm() > MAX_DISPLACEMENT) {
        v3_update /= (v3_update.norm() / MAX_DISPLACEMENT);
      }
      v1->position += v1_update;
      v2->position += v2_update;
      v3->position += v3_update;

      // update BVHTree
      HalfedgeIter h;
      h = v1->halfedge();
      for (int i = 0; i < v1->degree(); i++) {
        if (!h->isBoundary()) {
          h->face()->bvh_node->update_bvh(v1->position);
          h = h->twin()->next();
        }
      }
      h = v2->halfedge();
      for (int i = 0; i < v2->degree(); i++) {
        if (!h->isBoundary()) {
          h->face()->bvh_node->update_bvh(v2->position);
          h = h->twin()->next();
        }
      }
      h = v3->halfedge();
      for (int i = 0; i < v3->degree(); i++) {
        if (!h->isBoundary()) {
          h->face()->bvh_node->update_bvh(v3->position);
          h = h->twin()->next();
        }
      }
  }

  void Particle::dentFace(HardnessMap* hardness_map, BVHTree* bvh_tree) {
      VertexIter v1 = isect.face->halfedge()->vertex();
      VertexIter v2 = isect.face->halfedge()->next()->vertex();
      VertexIter v3 = isect.face->halfedge()->next()->next()->vertex();

      double area = isect.face->area();
      Vector3D energy = 0.5 * mass * velocity * velocity 
        * isect.barycentric / area; 

      Vector3D n = isect.face->normal();
      if (dot(direction, n) < 0) {
        n = -n;
      }
      // two displacements:
      //  Erosion from shearing off bits and pieces (move normal to face)
      //  Erosion from head-on collisions denting the rock (move in direction of particle)
      Vector3D v1_update = energy.x
        * (sin(acos(dot(direction, n))) / v1->hardness(hardness_map) / SHEAR_HARDNESS_BASE * n
          + dot(direction, n) / v1->hardness2(hardness_map) / FRONT_HARDNESS_BASE * direction);
      Vector3D v2_update = energy.y
        * (sin(acos(dot(direction, n))) / v2->hardness(hardness_map) / SHEAR_HARDNESS_BASE * n
          + dot(direction, n) / v2->hardness2(hardness_map) / FRONT_HARDNESS_BASE * direction);
      Vector3D v3_update = energy.z
        * (sin(acos(dot(direction, n))) / v3->hardness(hardness_map) / SHEAR_HARDNESS_BASE * n
          + dot(direction, n) / v3->hardness2(hardness_map) / FRONT_HARDNESS_BASE * direction);

      if (v1_update.norm() > MAX_DISPLACEMENT) {
        v1_update /= (v1_update.norm() / MAX_DISPLACEMENT);
      }
      if (v2_update.norm() > MAX_DISPLACEMENT) {
        v2_update /= (v2_update.norm() / MAX_DISPLACEMENT);
      }
      if (v3_update.norm() > MAX_DISPLACEMENT) {
        v3_update /= (v3_update.norm() / MAX_DISPLACEMENT);
      }
      v1->position += v1_update;
      v2->position += v2_update;
      v3->position += v3_update;

      // update BVHTree
      HalfedgeIter h;
      h = v1->halfedge();
      for (int i = 0; i < v1->degree(); i++) {
        if (!h->isBoundary()) {
          h->face()->bvh_node->update_bvh(v1->position);
          h = h->twin()->next();
        }
      }
      h = v2->halfedge();
      for (int i = 0; i < v2->degree(); i++) {
        if (!h->isBoundary()) {
          h->face()->bvh_node->update_bvh(v2->position);
          h = h->twin()->next();
        }
      }
      h = v3->halfedge();
      for (int i = 0; i < v3->degree(); i++) {
        if (!h->isBoundary()) {
          h->face()->bvh_node->update_bvh(v3->position);
          h = h->twin()->next();
        }
      }

      // correct creases
      correct_crease(
        isect.face,
        isect.face->halfedge()->twin()->face(),
        isect.face->halfedge()->vertex(),
        isect.face->halfedge()->next()->vertex()
      );
      correct_crease(
        isect.face,
        isect.face->halfedge()->next()->twin()->face(),
        isect.face->halfedge()->next()->vertex(),
        isect.face->halfedge()->next()->next()->vertex()
      );
      correct_crease(
        isect.face,
        isect.face->halfedge()->next()->next()->twin()->face(),
        isect.face->halfedge()->next()->next()->vertex(),
        isect.face->halfedge()->vertex()
      );
  }

  double Vertex::hardness(HardnessMap* hardness_map) {
    double hardness = 0.;
    for (int i = 0; i < hardness_map->max_depth-hardness_map->min_depth; i++) {
      int x0 = (int) ((position.x - hardness_map->start_pos.x) / hardness_map->scale[i]);
      int y0 = (int) ((position.y - hardness_map->start_pos.y) / hardness_map->scale[i]);
      int z0 = (int) ((position.z - hardness_map->start_pos.z) / hardness_map->scale[i]);
      x0 = std::max(std::min(x0, hardness_map->dim_x[i]-2), 0);
      y0 = std::max(std::min(y0, hardness_map->dim_y[i]-2), 0);
      z0 = std::max(std::min(z0, hardness_map->dim_z[i]-2), 0);
      int base_idx = z0*hardness_map->dim_y[i]*hardness_map->dim_x[i] + y0*hardness_map->dim_x[i] + x0;
      vector<Vector3D> vectors = vector<Vector3D>({
        hardness_map->vectors[i][base_idx],
        hardness_map->vectors[i][base_idx+1],
        hardness_map->vectors[i][base_idx+hardness_map->dim_x[i]],
        hardness_map->vectors[i][base_idx+hardness_map->dim_x[i]+1],
        hardness_map->vectors[i][base_idx+hardness_map->dim_y[i]*hardness_map->dim_x[i]],
        hardness_map->vectors[i][base_idx+hardness_map->dim_y[i]*hardness_map->dim_x[i]+1],
        hardness_map->vectors[i][base_idx+hardness_map->dim_y[i]*hardness_map->dim_x[i]+hardness_map->dim_x[i]],
        hardness_map->vectors[i][base_idx+hardness_map->dim_y[i]*hardness_map->dim_x[i]+hardness_map->dim_x[i]+1]
      });
      Vector3D base_pos = hardness_map->start_pos 
        + Vector3D(1, 0, 0)*hardness_map->scale[i]*x0
        + Vector3D(0, 1, 0)*hardness_map->scale[i]*y0
        + Vector3D(0, 0, 1)*hardness_map->scale[i]*z0;
      vector<Vector3D> corner_pos = vector<Vector3D>({
        base_pos,
        base_pos+Vector3D(1, 0, 0)*hardness_map->scale[i],
        base_pos+Vector3D(0, 1, 0)*hardness_map->scale[i],
        base_pos+Vector3D(1, 1, 0)*hardness_map->scale[i],
        base_pos+Vector3D(0, 0, 1)*hardness_map->scale[i],
        base_pos+Vector3D(1, 0, 1)*hardness_map->scale[i],
        base_pos+Vector3D(0, 1, 1)*hardness_map->scale[i],
        base_pos+Vector3D(1, 1, 1)*hardness_map->scale[i]
      });
      for (int j = 0; j < 8; j++) {
        hardness += (1.-abs(position.x-corner_pos[j].x)/hardness_map->scale[i])
          * (1.-abs(position.y-corner_pos[j].y)/hardness_map->scale[i])
          * (1.-abs(position.z-corner_pos[j].z)/hardness_map->scale[i])
          * dot((corner_pos[j]-position)/hardness_map->scale[i], vectors[j]);
      }
    }
    // return 40000. * exp(5.*tanh(hardness / sqrt(double(hardness_map->depth))));
    // return 20000. * exp(5.*(tanh(hardness / sqrt(double(hardness_map->depth)))+0.5));
    // return 2500. / pow(EPS_D + abs(tanh(hardness / sqrt(double(hardness_map->max_depth-hardness_map->min_depth)))), 2.);
    return 200000. * pow(abs(tanh(hardness / sqrt(double(hardness_map->max_depth-hardness_map->min_depth)))), 0.5);

  }

  double Vertex::hardness2(HardnessMap* hardness_map) {
    double hardness = 0.;
    for (int i = 0; i < hardness_map->max_depth-hardness_map->min_depth; i++) {
      int x0 = (int) ((position.x - hardness_map->start_pos.x) / hardness_map->scale[i]);
      int y0 = (int) ((position.y - hardness_map->start_pos.y) / hardness_map->scale[i]);
      int z0 = (int) ((position.z - hardness_map->start_pos.z) / hardness_map->scale[i]);
      x0 = std::max(std::min(x0, hardness_map->dim_x[i]-2), 0);
      y0 = std::max(std::min(y0, hardness_map->dim_y[i]-2), 0);
      z0 = std::max(std::min(z0, hardness_map->dim_z[i]-2), 0);
      int base_idx = z0*hardness_map->dim_y[i]*hardness_map->dim_x[i] + y0*hardness_map->dim_x[i] + x0;
      vector<Vector3D> vectors = vector<Vector3D>({
        hardness_map->vectors2[i][base_idx],
        hardness_map->vectors2[i][base_idx+1],
        hardness_map->vectors2[i][base_idx+hardness_map->dim_x[i]],
        hardness_map->vectors2[i][base_idx+hardness_map->dim_x[i]+1],
        hardness_map->vectors2[i][base_idx+hardness_map->dim_y[i]*hardness_map->dim_x[i]],
        hardness_map->vectors2[i][base_idx+hardness_map->dim_y[i]*hardness_map->dim_x[i]+1],
        hardness_map->vectors2[i][base_idx+hardness_map->dim_y[i]*hardness_map->dim_x[i]+hardness_map->dim_x[i]],
        hardness_map->vectors2[i][base_idx+hardness_map->dim_y[i]*hardness_map->dim_x[i]+hardness_map->dim_x[i]+1]
      });
      Vector3D base_pos = hardness_map->start_pos 
        + Vector3D(1, 0, 0)*hardness_map->scale[i]*x0
        + Vector3D(0, 1, 0)*hardness_map->scale[i]*y0
        + Vector3D(0, 0, 1)*hardness_map->scale[i]*z0;
      vector<Vector3D> corner_pos = vector<Vector3D>({
        base_pos,
        base_pos+Vector3D(1, 0, 0)*hardness_map->scale[i],
        base_pos+Vector3D(0, 1, 0)*hardness_map->scale[i],
        base_pos+Vector3D(1, 1, 0)*hardness_map->scale[i],
        base_pos+Vector3D(0, 0, 1)*hardness_map->scale[i],
        base_pos+Vector3D(1, 0, 1)*hardness_map->scale[i],
        base_pos+Vector3D(0, 1, 1)*hardness_map->scale[i],
        base_pos+Vector3D(1, 1, 1)*hardness_map->scale[i]
      });
      for (int j = 0; j < 8; j++) {
        hardness += (1.-abs(position.x-corner_pos[j].x)/hardness_map->scale[i])
          * (1.-abs(position.y-corner_pos[j].y)/hardness_map->scale[i])
          * (1.-abs(position.z-corner_pos[j].z)/hardness_map->scale[i])
          * dot((corner_pos[j]-position)/hardness_map->scale[i], vectors[j]);
      }
    }
    // return 40000. * exp(5.*tanh(hardness / sqrt(double(hardness_map->depth))));
    // return 20000. * exp(5.*(tanh(hardness / sqrt(double(hardness_map->depth)))+0.5));
    // return 2500. / pow(EPS_D + abs(tanh(hardness / sqrt(double(hardness_map->max_depth-hardness_map->min_depth)))), 2.);
    return 200000. * pow(abs(tanh(hardness / sqrt(double(hardness_map->max_depth-hardness_map->min_depth)))), 0.5);
  }

  bool Halfedge::isBoundary( void ) const
  // returns true if and only if this halfedge is on the boundary
  {
    return face()->isBoundary();
  }

  bool Edge::isBoundary( void ) const
  {
    return halfedge()->isBoundary() || halfedge()->twin()->isBoundary();
  }

  Vector3D Face::normal( void ) const
  {
    Vector3D N( 0., 0., 0. );

    HalfedgeCIter h = halfedge();
    do
    {
      Vector3D pi = h->vertex()->position;
      Vector3D pj = h->next()->vertex()->position;

      N += cross( pi, pj );

      h = h->next();
    }
    while( h != halfedge() );

    return N.unit();
  }

  double Face::area() {
    Vector3D p1 = halfedge()->vertex()->position;
    Vector3D p2 = halfedge()->next()->vertex()->position;
    Vector3D p3 = halfedge()->next()->next()->vertex()->position;

    Vector3D e1 = p2 - p1;
    Vector3D e2 = p3 - p1;
    double angle = acos(dot(e1.unit(), e2.unit()));
    return 0.5 * e1.norm() * e2.norm() * sin(angle);
  }

  void HalfedgeMesh :: build( const vector< vector<Index> >& polygons,
    const vector<Vector3D>& vertexPositions )
    // This method initializes the halfedge data structure from a raw list of polygons,
    // where each input polygon is specified as a list of vertex indices.  The input
    // must describe a manifold, oriented surface, where the orientation of a polygon
    // is determined by the order of vertices in the list.  Polygons must have at least
    // three vertices.  Note that there are no special conditions on the vertex indices,
    // i.e., they do not have to start at 0 or 1, nor does the collection of indices have
    // to be contiguous.  Overall, this initializer is designed to be robust but perhaps
    // not incredibly fast (though of course this does not affect the performance of the
    // resulting data structure).  One could also implement faster initializers that
    // handle important special cases (e.g., all triangles, or data that is known to be
    // manifold).
    //
    // Since there are no strong conditions on the indices of polygons, we assume that
    // the list of vertex positions is given in lexicographic order (i.e., that the
    // lowest index appearing in any polygon corresponds to the first entry of the list
    // of positions and so on).
    {
      // define some types, to improve readability
      typedef vector<Index> IndexList;
      typedef IndexList::const_iterator IndexListCIter;
      typedef vector<IndexList> PolygonList;
      typedef PolygonList::const_iterator PolygonListCIter;
      typedef pair<Index,Index> IndexPair; // ordered pair of vertex indices, corresponding to an edge of an oriented polygon

      // Clear any existing elements.
      halfedges.clear();
      vertices.clear();
      edges.clear();
      faces.clear();
      boundaries.clear();

      // Since the vertices in our halfedge mesh are stored in a linked list,
      // we will temporarily need to keep track of the correspondence between
      // indices of vertices in our input and pointers to vertices in the new
      // mesh (which otherwise can't be accessed by index).  Note that since
      // we're using a general-purpose map (rather than, say, a vector), we can
      // be a bit more flexible about the indexing scheme: input vertex indices
      // aren't required to be 0-based or 1-based; in fact, the set of indices
      // doesn't even have to be contiguous.  Taking advantage of this fact makes
      // our conversion a bit more robust to different types of input, including
      // data that comes from a subset of a full mesh.
      map<Index,VertexIter> indexToVertex; // maps a vertex index to the corresponding vertex

      // Also store the vertex degree, i.e., the number of polygons that use each
      // vertex; this information will be used to check that the mesh is manifold.
      map<VertexIter,Size> vertexDegree;

      // First, we do some basic sanity checks on the input.
      for( PolygonListCIter p = polygons.begin(); p != polygons.end(); p++ )
      {
        if( p->size() < 3 )
        {
          // Refuse to build the mesh if any of the polygons have fewer than three vertices.
          // (Note that if we omit this check the code will still construct something fairly
          // meaningful for 1- and 2-point polygons, but enforcing this stricter requirement
          // on the input will help simplify code further downstream, since it can be certain
          // it doesn't have to check for these rather degenerate cases.)
          cerr << "Error converting polygons to halfedge mesh: each polygon must have at least three vertices." << endl;
          exit( 1 );
        }

        // We want to count the number of distinct vertex indices in this
        // polygon, to make sure it's the same as the number of vertices
        // in the polygon---if they disagree, then the polygon is not valid
        // (or at least, for simplicity we don't handle polygons of this type!).
        set<Index> polygonIndices;

        // loop over polygon vertices
        for( IndexListCIter i = p->begin(); i != p->end(); i++ )
        {
          polygonIndices.insert( *i );

          // allocate one vertex for each new index we encounter
          if( indexToVertex.find( *i ) == indexToVertex.end() )
          {
            VertexIter v = newVertex();
            v->halfedge() = halfedges.end(); // this vertex doesn't yet point to any halfedge
            indexToVertex[ *i ] = v;
            vertexDegree[ v ] = 1; // we've now seen this vertex only once
          }
          else
          {
            // keep track of the number of times we've seen this vertex
            vertexDegree[ indexToVertex[ *i ] ]++;
          }

        } // end loop over polygon vertices

        // check that all vertices of the current polygon are distinct
        Size degree = p->size(); // number of vertices in this polygon
        if( polygonIndices.size() < degree )
        {
          cerr << "Error converting polygons to halfedge mesh: one of the input polygons does not have distinct vertices!" << endl;
          cerr << "(vertex indices:";
          for( IndexListCIter i = p->begin(); i != p->end(); i++ )
          {
            cerr << " " << *i;
          }
          cerr << ")" << endl;
          exit( 1 );
        } // end check that polygon vertices are distinct

      } // end basic sanity checks on input

      // The number of vertices in the mesh is the
      // number of unique indices seen in the input.
      Size nVertices = indexToVertex.size();

      // The number of faces is just the number of polygons in the input.
      Size nFaces = polygons.size();
      faces.resize( nFaces ); // allocate storage for faces in our new mesh

      // We will store a map from ordered pairs of vertex indices to
      // the corresponding halfedge object in our new (halfedge) mesh;
      // this map gets constructed during the next loop over polygons.
      map<IndexPair,HalfedgeIter> pairToHalfedge;

      // Next, we actually build the halfedge connectivity by again looping over polygons
      PolygonListCIter p;
      FaceIter f;
      for( p = polygons.begin(), f = faces.begin();
      p != polygons.end();
      p++, f++ )
      {
        vector<HalfedgeIter> faceHalfedges; // cyclically ordered list of the half edges of this face
        Size degree = p->size(); // number of vertices in this polygon

        // loop over the halfedges of this face (equivalently, the ordered pairs of consecutive vertices)
        for( Index i = 0; i < degree; i++ )
        {
          Index a = (*p)[i]; // current index
          Index b = (*p)[(i+1)%degree]; // next index, in cyclic order
          IndexPair ab( a, b );
          HalfedgeIter hab;

          // check if this halfedge already exists; if so, we have a problem!
          if( pairToHalfedge.find( ab ) != pairToHalfedge.end() )
          {
            cerr << "Error converting polygons to halfedge mesh: found multiple oriented edges with indices (" << a << ", " << b << ")." << endl;
            cerr << "This means that either (i) more than two faces contain this edge (hence the surface is nonmanifold), or" << endl;
            cerr << "(ii) there are exactly two faces containing this edge, but they have the same orientation (hence the surface is" << endl;
            cerr << "not consistently oriented." << endl;
            exit( 1 );
          }
          else // otherwise, the halfedge hasn't been allocated yet
          {
            // so, we point this vertex pair to a new halfedge
            hab = newHalfedge();
            pairToHalfedge[ab] = hab;

            // link the new halfedge to its face
            hab->face() = f;
            hab->face()->halfedge() = hab;

            // also link it to its starting vertex
            hab->vertex() = indexToVertex[a];
            hab->vertex()->halfedge() = hab;

            // keep a list of halfedges in this face, so that we can later
            // link them together in a loop (via their "next" pointers)
            faceHalfedges.push_back( hab );
          }

          // Also, check if the twin of this halfedge has already been constructed (during
          // construction of a different face).  If so, link the twins together and allocate
          // their shared halfedge.  By the end of this pass over polygons, the only halfedges
          // that will not have a twin will hence be those that sit along the domain boundary.
          IndexPair ba( b, a );
          map<IndexPair,HalfedgeIter>::iterator iba = pairToHalfedge.find( ba );
          if( iba != pairToHalfedge.end() )
          {
            HalfedgeIter hba = iba->second;

            // link the twins
            hab->twin() = hba;
            hba->twin() = hab;

            // allocate and link their edge
            EdgeIter e = newEdge();
            hab->edge() = e;
            hba->edge() = e;
            e->halfedge() = hab;
          }
          else // If we didn't find a twin...
          {
            // ...mark this halfedge as being twinless by pointing
            // it to the end of the list of halfedges. If it remains
            // twinless by the end of the current loop over polygons,
            // it will be linked to a boundary face in the next pass.
            hab->twin() = halfedges.end();
          }

        } // end loop over the current polygon's halfedges

        // Now that all the halfedges of this face have been allocated,
        // we can link them together via their "next" pointers.
        for( Index i = 0; i < degree; i++ )
        {
          Index j = (i+1) % degree; // index of the next halfedge, in cyclic order
          faceHalfedges[i]->next() = faceHalfedges[j];
        }

      } // done building basic halfedge connectivity

      // For each vertex on the boundary, advance its halfedge pointer to one that is also on the boundary.
      for( VertexIter v = verticesBegin(); v != verticesEnd(); v++ )
      {
        // loop over halfedges around vertex
        HalfedgeIter h = v->halfedge();
        do
        {
          if( h->twin() == halfedges.end() )
          {
            v->halfedge() = h;
            break;
          }

          h = h->twin()->next();
        }
        while( h != v->halfedge() ); // end loop over halfedges around vertex

      } // done advancing halfedge pointers for boundary vertices

      // Next we construct new faces for each boundary component.
      for( HalfedgeIter h = halfedgesBegin(); h != halfedgesEnd(); h++ ) // loop over all halfedges
      {
        // Any halfedge that does not yet have a twin is on the boundary of the domain.
        // If we follow the boundary around long enough we will of course eventually make a
        // closed loop; we can represent this boundary loop by a new face. To make clear the
        // distinction between faces and boundary loops, the boundary face will (i) have a flag
        // indicating that it is a boundary loop, and (ii) be stored in a list of boundaries,
        // rather than the usual list of faces.  The reason we need the both the flag *and* the
        // separate list is that faces are often accessed in two fundamentally different ways:
        // either by (i) local traversal of the neighborhood of some mesh element using the
        // halfedge structure, or (ii) global traversal of all faces (or boundary loops).
        if( h->twin() == halfedges.end() )
        {
          FaceIter b = newBoundary();
          vector<HalfedgeIter> boundaryHalfedges; // keep a list of halfedges along the boundary, so we can link them together

          // We now need to walk around the boundary, creating new
          // halfedges and edges along the boundary loop as we go.
          HalfedgeIter i = h;
          do
          {
            // create a twin, which becomes a halfedge of the boundary loop
            HalfedgeIter t = newHalfedge();
            boundaryHalfedges.push_back( t ); // keep a list of all boundary halfedges, in cyclic order
            i->twin() = t;
            t->twin() = i;
            t->face() = b;
            t->vertex() = i->next()->vertex();

            // create the shared edge
            EdgeIter e = newEdge();
            e->halfedge() = i;
            i->edge() = e;
            t->edge() = e;

            // Advance i to the next halfedge along the current boundary loop
            // by walking around its target vertex and stopping as soon as we
            // find a halfedge that does not yet have a twin defined.
            i = i->next();
            while( i != h && // we're done if we end up back at the beginning of the loop
            i->twin() != halfedges.end() ) // otherwise, we're looking for the next twinless halfedge along the loop
            {
              i = i->twin();
              i = i->next();
            }
          }
          while( i != h );

          // The only pointers that still need to be set are the "next" pointers of the twins;
          // these we can set from the list of boundary halfedges, but we must use the opposite
          // order from the order in the list, since the orientation of the boundary loop is
          // opposite the orientation of the halfedges "inside" the domain boundary.
          Size degree = boundaryHalfedges.size();
          for( Index p = 0; p < degree; p++ )
          {
            Index q = (p-1+degree) % degree;
            boundaryHalfedges[p]->next() = boundaryHalfedges[q];
          }

        } // end construction of one of the boundary loops

        // Note that even though we are looping over all halfedges, we will still construct
        // the appropriate number of boundary loops (and not, say, one loop per boundary
        // halfedge).  The reason is that as we continue to iterate through halfedges, we
        // check whether their twin has been assigned, and since new twins may have been
        // assigned earlier in this loop, we will end up skipping many subsequent halfedges.

      } // done adding "virtual" faces corresponding to boundary loops

      // To make later traversal of the mesh easier, we will now advance the halfedge
      // associated with each vertex such that it refers to the *first* non-boundary
      // halfedge, rather than the last one.
      for( VertexIter v = verticesBegin(); v != verticesEnd(); v++ )
      {
        v->halfedge() = v->halfedge()->twin()->next();
      }

      // Finally, we check that all vertices are manifold.
      for( VertexIter v = vertices.begin(); v != vertices.end(); v++ )
      {
        // First check that this vertex is not a "floating" vertex;
        // if it is then we do not have a valid 2-manifold surface.
        if( v->halfedge() == halfedges.end() )
        {
          cerr << "Error converting polygons to halfedge mesh: some vertices are not referenced by any polygon." << endl;
          exit( 1 );
        }

        // Next, check that the number of halfedges emanating from this vertex in our half
        // edge data structure equals the number of polygons containing this vertex, which
        // we counted during our first pass over the mesh.  If not, then our vertex is not
        // a "fan" of polygons, but instead has some other (nonmanifold) structure.
        Size count = 0;
        HalfedgeIter h = v->halfedge();
        do
        {
          if( !h->face()->isBoundary() )
          {
            count++;
          }
          h = h->twin()->next();
        }
        while( h != v->halfedge() );

        if( count != vertexDegree[v] )
        {
          cerr << "Error converting polygons to halfedge mesh: at least one of the vertices is nonmanifold." << endl;
          exit( 1 );
        }
      } // end loop over vertices

      // Now that we have the connectivity, we copy the list of vertex
      // positions into member variables of the individual vertices.
      if( vertexPositions.size() != vertices.size() )
      {
        cerr << "Error converting polygons to halfedge mesh: number of vertex positions is different from the number of distinct vertices!" << endl;
        cerr << "(number of positions in input: " << vertexPositions.size() << ")" << endl;
        cerr << "(  number of vertices in mesh: " << vertices.size() << ")" << endl;
        exit( 1 );
      }
      // Since an STL map internally sorts its keys, we can iterate over the map from vertex indices to
      // vertex iterators to visit our (input) vertices in lexicographic order
      int i = 0;
      for( map<Index,VertexIter>::const_iterator e = indexToVertex.begin(); e != indexToVertex.end(); e++ )
      {
        // grab a pointer to the vertex associated with the current key (i.e., the current index)
        VertexIter v = e->second;

        // set the position of this vertex to the corresponding position in the input
        v->position = vertexPositions[ i ];
        i++;
      }

    } // end HalfedgeMesh::build()

    const HalfedgeMesh& HalfedgeMesh :: operator=( const HalfedgeMesh& mesh )
    // The assignment operator does a "deep" copy of the halfedge mesh data structure; in
    // other words, it makes new instances of each mesh element, and ensures that pointers
    // in the copy point to the newly allocated elements rather than elements in the original
    // mesh.  This behavior is especially important for making assignments, since the mesh
    // on the right-hand side of an assignment may be temporary (hence any pointers to elements
    // in this mesh will become invalid as soon as it is released.)
    {
      // Clear any existing elements.
      halfedges.clear();
      vertices.clear();
      edges.clear();
      faces.clear();
      boundaries.clear();

      // These maps will be used to identify elements of the old mesh
      // with elements of the new mesh.  (Note that we can use a single
      // map for both interior and boundary faces, because the map
      // doesn't care which list of faces these iterators come from.)
      map< HalfedgeCIter, HalfedgeIter > halfedgeOldToNew;
      map<   VertexCIter,   VertexIter >   vertexOldToNew;
      map<     EdgeCIter,     EdgeIter >     edgeOldToNew;
      map<     FaceCIter,     FaceIter >     faceOldToNew;

      // Copy geometry from the original mesh and create a map from
      // pointers in the original mesh to those in the new mesh.
      for( HalfedgeCIter h =      mesh.halfedgesBegin(); h !=  mesh.halfedgesEnd(); h++ ) halfedgeOldToNew[ h ] =  halfedges.insert(  halfedges.end(), *h );
      for(   VertexCIter v =       mesh.verticesBegin(); v !=   mesh.verticesEnd(); v++ )   vertexOldToNew[ v ] =   vertices.insert(   vertices.end(), *v );
      for(     EdgeCIter e =          mesh.edgesBegin(); e !=      mesh.edgesEnd(); e++ )     edgeOldToNew[ e ] =      edges.insert(      edges.end(), *e );
      for(     FaceCIter f =          mesh.facesBegin(); f !=      mesh.facesEnd(); f++ )     faceOldToNew[ f ] =      faces.insert(      faces.end(), *f );
      for(     FaceCIter b =     mesh.boundariesBegin(); b != mesh.boundariesEnd(); b++ )     faceOldToNew[ b ] = boundaries.insert( boundaries.end(), *b );

      // "Search and replace" old pointers with new ones.
      for( HalfedgeIter he = halfedgesBegin(); he != halfedgesEnd(); he++ )
      {
        he->next()   = halfedgeOldToNew[ he->next()   ];
        he->twin()   = halfedgeOldToNew[ he->twin()   ];
        he->vertex() =   vertexOldToNew[ he->vertex() ];
        he->edge()   =     edgeOldToNew[ he->edge()   ];
        he->face()   =     faceOldToNew[ he->face()   ];
      }
      for( VertexIter v =   verticesBegin(); v !=   verticesEnd(); v++ ) v->halfedge() = halfedgeOldToNew[ v->halfedge() ];
      for(   EdgeIter e =      edgesBegin(); e !=      edgesEnd(); e++ ) e->halfedge() = halfedgeOldToNew[ e->halfedge() ];
      for(   FaceIter f =      facesBegin(); f !=      facesEnd(); f++ ) f->halfedge() = halfedgeOldToNew[ f->halfedge() ];
      for(   FaceIter b = boundariesBegin(); b != boundariesEnd(); b++ ) b->halfedge() = halfedgeOldToNew[ b->halfedge() ];

      // Return a reference to the new mesh.
      return *this;
    }

    HalfedgeMesh :: HalfedgeMesh( const HalfedgeMesh& mesh )
    {
      *this = mesh;
    }
  } // End of CMU 462 namespace.
