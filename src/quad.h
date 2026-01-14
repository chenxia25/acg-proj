#ifndef QUAD_H
#define QUAD_H

#include <cmath>

#include "hittable.h"
#include "vec3.h"

struct Quad : public Hittable {
  Point3 Q;
  Vec3 u, v;
  shared_ptr<Material> mat_ptr;
  AABB box;
  Vec3 normal;
  double D;
  double area;

  Quad(const Point3& _Q, const Vec3& _u, const Vec3& _v, shared_ptr<Material> m)
      : Q(_Q), u(_u), v(_v), mat_ptr(m) {
    auto n = cross(u, v);
    normal = unit_vector(n);
    D = dot(normal, Q);
    area = n.length();
    set_bounding_box();
  }

  virtual void set_bounding_box() {
    auto bbox_diagonal1 = AABB(Q, Q + u + v);
    auto bbox_diagonal2 = AABB(Q + u, Q + v);
    box = surrounding_box(bbox_diagonal1, bbox_diagonal2);

    Point3 min_pt = box.min();
    Point3 max_pt = box.max();
    double epsilon = 0.0001;
    for (int i = 0; i < 3; i++) {
      if (std::abs(max_pt[i] - min_pt[i]) < epsilon) {
        max_pt[i] += epsilon;
        min_pt[i] -= epsilon;
      }
    }
    box = AABB(min_pt, max_pt);
  }

  virtual bool bounding_box(double time0, double time1,
                            AABB& output_box) const override {
    output_box = box;
    return true;
  }

  virtual bool hit(const Ray& r, double t_min, double t_max,
                   HitRecord& rec) const override {
    auto denom = dot(normal, r.direction());
    if (std::abs(denom) < 1e-8) return false;

    auto t = (D - dot(normal, r.origin())) / denom;
    if (t < t_min || t > t_max) return false;

    auto intersection = r.at(t);
    Vec3 planar_hitpt_vector = intersection - Q;

    auto alpha = dot(normal, cross(planar_hitpt_vector, v));
    auto beta = dot(normal, cross(u, planar_hitpt_vector));

    if (area < 1e-8) return false;

    alpha /= area;
    beta /= area;

    if (alpha < 0 || alpha > 1 || beta < 0 || beta > 1) return false;

    rec.t = t;
    rec.p = intersection;
    rec.mat_ptr = mat_ptr;
    rec.set_face_normal(r, normal);
    return true;
  }

  virtual double pdf_value(const Point3& origin, const Vec3& v) const override {
    HitRecord rec;
    if (!this->hit(Ray(origin, v), 0.001, infinity, rec)) return 0;

    auto distance_squared = rec.t * rec.t * v.length_squared();
    auto cosine = std::abs(dot(v, rec.normal) / v.length());

    return distance_squared / (cosine * area);
  }

  virtual Vec3 random(const Point3& origin) const override {
    auto p = Q + (random_double() * u) + (random_double() * v);
    return p - origin;
  }
};

#endif