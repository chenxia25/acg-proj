#ifndef HITTABLE_H
#define HITTABLE_H

#include "aabb.h"
#include "ray.h"

struct Material;

struct HitRecord {
  Point3 p;
  Vec3 normal;
  shared_ptr<Material> mat_ptr;
  double t;
  bool front_face;

  inline void set_face_normal(const Ray& r, const Vec3& outward_normal) {
    front_face = dot(r.direction(), outward_normal) < 0;
    normal = front_face ? outward_normal : -outward_normal;
  }
};

struct Hittable {
  virtual ~Hittable() = default;

  virtual bool hit(const Ray& r, double t_min, double t_max,
                   HitRecord& rec) const = 0;
  virtual bool bounding_box(double time0, double time1,
                            AABB& output_box) const = 0;

  virtual double pdf_value(const Point3& o, const Vec3& v) const { return 0.0; }

  virtual Vec3 random(const Point3& o) const { return Vec3(1, 0, 0); }
};

#endif