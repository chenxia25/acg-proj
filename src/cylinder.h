#ifndef CYLINDER_H
#define CYLINDER_H

#include "hittable.h"
#include "vec3.h"

class Cylinder : public Hittable {
 public:
  Point3 center_bottom;
  double height;
  double radius;
  shared_ptr<Material> mat_ptr;

  Cylinder(Point3 _cb, double _h, double _r, shared_ptr<Material> _m)
      : center_bottom(_cb), height(_h), radius(_r), mat_ptr(_m) {};

  virtual bool hit(const Ray& r, double t_min, double t_max,
                   HitRecord& rec) const override {
    Vec3 oc = r.origin() - center_bottom;
    auto a = r.direction().x() * r.direction().x() +
             r.direction().z() * r.direction().z();
    auto b = 2.0 * (oc.x() * r.direction().x() + oc.z() * r.direction().z());
    auto c = oc.x() * oc.x() + oc.z() * oc.z() - radius * radius;

    auto discriminant = b * b - 4 * a * c;
    if (discriminant < 0) return false;
    auto sqrtd = sqrt(discriminant);

    auto root = (-b - sqrtd) / (2 * a);
    if (root < t_min || root > t_max) {
      root = (-b + sqrtd) / (2 * a);
      if (root < t_min || root > t_max) return false;
    }

    auto p = r.at(root);
    if (p.y() < center_bottom.y() || p.y() > center_bottom.y() + height) {
      return false;
    }

    rec.t = root;
    rec.p = p;
    Vec3 outward_normal =
        Vec3(p.x() - center_bottom.x(), 0, p.z() - center_bottom.z());
    rec.set_face_normal(r, unit_vector(outward_normal));
    rec.mat_ptr = mat_ptr;

    return true;
  }

  virtual bool bounding_box(double time0, double time1,
                            AABB& output_box) const override {
    Point3 min_pt(center_bottom.x() - radius, center_bottom.y(),
                  center_bottom.z() - radius);
    Point3 max_pt(center_bottom.x() + radius, center_bottom.y() + height,
                  center_bottom.z() + radius);
    output_box = AABB(min_pt, max_pt);
    return true;
  }
};

#endif