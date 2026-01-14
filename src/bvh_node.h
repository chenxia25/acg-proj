#ifndef BVH_NODE_H
#define BVH_NODE_H

#include <algorithm>
#include <vector>

#include "hittable.h"
#include "hittable_list.h"
#include "utility.h"

inline bool box_compare(const shared_ptr<Hittable> a,
                        const shared_ptr<Hittable> b, int axis) {
  AABB box_a, box_b;
  if (!a->bounding_box(0, 0, box_a) || !b->bounding_box(0, 0, box_b))
    std::cerr << "No bounding box in bvh_node constructor.\n";
  return box_a.min().e[axis] < box_b.min().e[axis];
}

inline bool box_x_compare(const shared_ptr<Hittable> a,
                          const shared_ptr<Hittable> b) {
  return box_compare(a, b, 0);
}
inline bool box_y_compare(const shared_ptr<Hittable> a,
                          const shared_ptr<Hittable> b) {
  return box_compare(a, b, 1);
}
inline bool box_z_compare(const shared_ptr<Hittable> a,
                          const shared_ptr<Hittable> b) {
  return box_compare(a, b, 2);
}

struct BvhNode : public Hittable {
  shared_ptr<Hittable> left;
  shared_ptr<Hittable> right;
  AABB box;

  BvhNode() {}

  BvhNode(const HittableList& list, double time0, double time1)
      : BvhNode(list.objects, 0, list.objects.size(), time0, time1) {}

  BvhNode(const std::vector<shared_ptr<Hittable>>& src_objects, size_t start,
          size_t end, double time0, double time1) {
    auto objects = src_objects;
    size_t object_span = end - start;

    AABB centroid_bounds;
    AABB total_bounds;

    for (size_t i = start; i < end; ++i) {
      AABB temp_box;
      if (!objects[i]->bounding_box(time0, time1, temp_box)) continue;

      total_bounds =
          (i == start) ? temp_box : surrounding_box(total_bounds, temp_box);

      Point3 center = 0.5 * (temp_box.min() + temp_box.max());
      if (i == start) {
        centroid_bounds = AABB(center, center);
      } else {
        centroid_bounds =
            surrounding_box(centroid_bounds, AABB(center, center));
      }
    }

    int axis = 0;

    if (object_span <= 4) {
      Vec3 extent = centroid_bounds.max() - centroid_bounds.min();
      if (extent.y() > extent.x()) axis = 1;
      if (extent.z() > extent.y() && extent.z() > extent.x()) axis = 2;

      auto comparator = (axis == 0)   ? box_x_compare
                        : (axis == 1) ? box_y_compare
                                      : box_z_compare;
      std::sort(objects.begin() + start, objects.begin() + end, comparator);

      if (object_span == 1) {
        left = right = objects[start];
      } else if (object_span == 2) {
        left = objects[start];
        right = objects[start + 1];
      } else {
        auto mid = start + object_span / 2;
        left = make_shared<BvhNode>(objects, start, mid, time0, time1);
        right = make_shared<BvhNode>(objects, mid, end, time0, time1);
      }
    } else {
      int best_axis = -1;
      int best_split_bucket = -1;
      double min_cost = infinity;
      const int nBuckets = 12;

      for (int a = 0; a < 3; ++a) {
        Vec3 extent = centroid_bounds.max() - centroid_bounds.min();
        if (extent[a] < 1e-4) continue;

        struct BucketInfo {
          int count = 0;
          AABB bounds;
        };
        BucketInfo buckets[nBuckets];

        for (size_t i = start; i < end; ++i) {
          AABB temp_box;
          objects[i]->bounding_box(time0, time1, temp_box);
          Point3 center = 0.5 * (temp_box.min() + temp_box.max());

          double offset = (center[a] - centroid_bounds.min()[a]) / extent[a];
          int b = static_cast<int>(nBuckets * offset);
          if (b == nBuckets) b = nBuckets - 1;

          buckets[b].count++;
          if (buckets[b].count == 1)
            buckets[b].bounds = temp_box;
          else
            buckets[b].bounds = surrounding_box(buckets[b].bounds, temp_box);
        }

        for (int i = 0; i < nBuckets - 1; ++i) {
          AABB b0, b1;
          int count0 = 0, count1 = 0;

          for (int j = 0; j <= i; ++j) {
            if (buckets[j].count > 0) {
              if (count0 == 0)
                b0 = buckets[j].bounds;
              else
                b0 = surrounding_box(b0, buckets[j].bounds);
              count0 += buckets[j].count;
            }
          }

          for (int j = i + 1; j < nBuckets; ++j) {
            if (buckets[j].count > 0) {
              if (count1 == 0)
                b1 = buckets[j].bounds;
              else
                b1 = surrounding_box(b1, buckets[j].bounds);
              count1 += buckets[j].count;
            }
          }

          if (count0 == 0 || count1 == 0) continue;

          auto surface_area = [](const AABB& b) {
            auto d = b.max() - b.min();
            return d.x() * d.y() + d.y() * d.z() + d.z() * d.x();
          };

          double cost =
              1 + (count0 * surface_area(b0) + count1 * surface_area(b1)) /
                      surface_area(total_bounds);

          if (cost < min_cost) {
            min_cost = cost;
            best_axis = a;
            best_split_bucket = i;
          }
        }
      }

      if (best_axis != -1) {
        auto comparator = (best_axis == 0)   ? box_x_compare
                          : (best_axis == 1) ? box_y_compare
                                             : box_z_compare;

        auto mid_ptr = std::partition(
            objects.begin() + start, objects.begin() + end,
            [&](const shared_ptr<Hittable> obj) {
              AABB temp_box;
              obj->bounding_box(time0, time1, temp_box);
              Point3 center = 0.5 * (temp_box.min() + temp_box.max());

              Vec3 extent = centroid_bounds.max() - centroid_bounds.min();
              double offset =
                  (center[best_axis] - centroid_bounds.min()[best_axis]) /
                  extent[best_axis];
              int b = static_cast<int>(nBuckets * offset);
              if (b == nBuckets) b = nBuckets - 1;

              return b <= best_split_bucket;
            });

        size_t mid = mid_ptr - objects.begin();

        if (mid == start || mid == end) {
          mid = start + object_span / 2;
          std::sort(objects.begin() + start, objects.begin() + end, comparator);
        }

        left = make_shared<BvhNode>(objects, start, mid, time0, time1);
        right = make_shared<BvhNode>(objects, mid, end, time0, time1);
      } else {
        auto comparator = box_x_compare;
        std::sort(objects.begin() + start, objects.begin() + end, comparator);
        auto mid = start + object_span / 2;
        left = make_shared<BvhNode>(objects, start, mid, time0, time1);
        right = make_shared<BvhNode>(objects, mid, end, time0, time1);
      }
    }

    AABB box_left, box_right;
    if (!left->bounding_box(time0, time1, box_left) ||
        !right->bounding_box(time0, time1, box_right))
      std::cerr << "No bounding box in bvh_node constructor.\n";

    box = surrounding_box(box_left, box_right);
  }

  virtual bool hit(const Ray& r, double t_min, double t_max,
                   HitRecord& rec) const override {
    if (!box.hit(r, t_min, t_max)) return false;

    bool hit_left = left->hit(r, t_min, t_max, rec);
    bool hit_right = right->hit(r, t_min, hit_left ? rec.t : t_max, rec);

    return hit_left || hit_right;
  }

  virtual bool bounding_box(double time0, double time1,
                            AABB& output_box) const override {
    output_box = box;
    return true;
  }
};

#endif