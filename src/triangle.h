#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "hittable.h"
#include "vec3.h"

struct Triangle : public Hittable {
  Point3 v0, v1, v2;
  shared_ptr<Material> mat_ptr;

  Triangle(Point3 _v0, Point3 _v1, Point3 _v2, shared_ptr<Material> m)
      : v0(_v0), v1(_v1), v2(_v2), mat_ptr(m) {}

  virtual bool hit(const Ray& r, double t_min, double t_max,
                   HitRecord& rec) const override {
    Vec3 v0v1 = v1 - v0;
    Vec3 v0v2 = v2 - v0;
    Vec3 pvec = cross(r.direction(), v0v2);
    auto det = dot(v0v1, pvec);

    if (std::abs(det) < 1e-8) return false;

    auto inv_det = 1.0 / det;
    Vec3 tvec = r.origin() - v0;
    auto u = dot(tvec, pvec) * inv_det;
    if (u < 0 || u > 1) return false;

    Vec3 qvec = cross(tvec, v0v1);
    auto v = dot(r.direction(), qvec) * inv_det;
    if (v < 0 || u + v > 1) return false;

    auto t = dot(v0v2, qvec) * inv_det;

    if (t < t_min || t > t_max) return false;

    rec.t = t;
    rec.p = r.at(t);
    Vec3 normal = unit_vector(cross(v0v1, v0v2));
    rec.set_face_normal(r, normal);
    rec.mat_ptr = mat_ptr;

    return true;
  }

  virtual bool bounding_box(double time0, double time1,
                            AABB& output_box) const override {
    double min_x = fmin(v0.x(), fmin(v1.x(), v2.x()));
    double min_y = fmin(v0.y(), fmin(v1.y(), v2.y()));
    double min_z = fmin(v0.z(), fmin(v1.z(), v2.z()));

    double max_x = fmax(v0.x(), fmax(v1.x(), v2.x()));
    double max_y = fmax(v0.y(), fmax(v1.y(), v2.y()));
    double max_z = fmax(v0.z(), fmax(v1.z(), v2.z()));

    double epsilon = 1e-4;
    output_box =
        AABB(Point3(min_x - epsilon, min_y - epsilon, min_z - epsilon),
             Point3(max_x + epsilon, max_y + epsilon, max_z + epsilon));
    return true;
  }
};

#endif