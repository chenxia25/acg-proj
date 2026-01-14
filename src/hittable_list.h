#ifndef HITTABLE_LIST_H
#define HITTABLE_LIST_H

#include <memory>
#include <vector>

#include "hittable.h"

using std::make_shared;
using std::shared_ptr;

struct HittableList : public Hittable {
  std::vector<shared_ptr<Hittable>> objects;

  HittableList() {}
  HittableList(shared_ptr<Hittable> object) { add(object); }

  void clear() { objects.clear(); }
  void add(shared_ptr<Hittable> object) { objects.push_back(object); }

  virtual bool hit(const Ray& r, double t_min, double t_max,
                   HitRecord& rec) const override {
    HitRecord temp_rec;
    bool hit_anything = false;
    auto closest_so_far = t_max;

    for (const auto& object : objects) {
      if (object->hit(r, t_min, closest_so_far, temp_rec)) {
        hit_anything = true;
        closest_so_far = temp_rec.t;
        rec = temp_rec;
      }
    }
    return hit_anything;
  }

  virtual bool bounding_box(double time0, double time1,
                            AABB& output_box) const override {
    if (objects.empty()) return false;

    AABB temp_box;
    bool first_box = true;

    for (const auto& object : objects) {
      if (!object->bounding_box(time0, time1, temp_box)) return false;
      output_box = first_box ? temp_box : surrounding_box(output_box, temp_box);
      first_box = false;
    }
    return true;
  }

  virtual double pdf_value(const Point3& o, const Vec3& v) const override {
    auto weight = 1.0 / objects.size();
    auto sum = 0.0;
    for (const auto& object : objects) sum += weight * object->pdf_value(o, v);
    return sum;
  }

  virtual Vec3 random(const Point3& o) const override {
    auto int_size = static_cast<int>(objects.size());
    return objects[random_double(0, int_size)]->random(o);
  }
};

#endif