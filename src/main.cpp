#include <algorithm>
#include <atomic>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "bvh_node.h"
#include "cylinder.h"
#include "hittable.h"
#include "hittable_list.h"
#include "material.h"
#include "quad.h"
#include "triangle.h"
#include "utility.h"

shared_ptr<HittableList> load_obj(const std::string& filename, Point3 offset,
                                  double scale, double rotate_y_degrees,
                                  shared_ptr<Material> mat) {
  auto objects = make_shared<HittableList>();
  std::vector<Point3> vertices;
  std::ifstream file(filename);

  if (!file.is_open()) {
    std::cerr << "ERROR: Could not open file " << filename << "\n";
    return objects;
  }

  std::vector<Point3> raw_vertices;
  std::string line;
  while (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string prefix;
    ss >> prefix;
    if (prefix == "v") {
      double x, y, z;
      ss >> x >> y >> z;
      raw_vertices.push_back(Point3(x, y, z));
    }
  }

  if (raw_vertices.empty()) return objects;
  Point3 min_pt(infinity, infinity, infinity);
  Point3 max_pt(-infinity, -infinity, -infinity);
  for (const auto& v : raw_vertices) {
    min_pt = Point3(fmin(min_pt.x(), v.x()), fmin(min_pt.y(), v.y()),
                    fmin(min_pt.z(), v.z()));
    max_pt = Point3(fmax(max_pt.x(), v.x()), fmax(max_pt.y(), v.y()),
                    fmax(max_pt.z(), v.z()));
  }

  double center_x = (min_pt.x() + max_pt.x()) / 2.0;
  double min_y = min_pt.y();
  double center_z = (min_pt.z() + max_pt.z()) / 2.0;

  double theta = degrees_to_radians(rotate_y_degrees);
  double cos_t = cos(theta);
  double sin_t = sin(theta);

  for (const auto& v : raw_vertices) {
    double x = v.x() - center_x;
    double y = v.y() - min_y;
    double z = v.z() - center_z;

    double x_rot = x * cos_t + z * sin_t;
    double z_rot = -x * sin_t + z * cos_t;

    vertices.push_back(Point3(x_rot * scale + offset.x(),
                              y * scale + offset.y(),
                              z_rot * scale + offset.z()));
  }

  file.clear();
  file.seekg(0, std::ios::beg);
  while (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string prefix;
    ss >> prefix;

    if (prefix == "f") {
      int v_idx[3];
      for (int i = 0; i < 3; ++i) {
        std::string vertex_str;
        ss >> vertex_str;
        size_t slash_pos = vertex_str.find('/');
        if (slash_pos != std::string::npos) {
          v_idx[i] = std::stoi(vertex_str.substr(0, slash_pos)) - 1;
        } else {
          v_idx[i] = std::stoi(vertex_str) - 1;
        }
      }
      objects->add(make_shared<Triangle>(vertices[v_idx[0]], vertices[v_idx[1]],
                                         vertices[v_idx[2]], mat));
    }
  }
  return objects;
}

Vec3 aces_approx(Vec3 v) {
  v *= 0.85;
  double a = 2.51f;
  double b = 0.03f;
  double c = 2.43f;
  double d = 0.59f;
  double e = 0.14f;
  return Vec3((v.x() * (a * v.x() + b)) / (v.x() * (c * v.x() + d) + e),
              (v.y() * (a * v.y() + b)) / (v.y() * (c * v.y() + d) + e),
              (v.z() * (a * v.z() + b)) / (v.z() * (c * v.z() + d) + e));
}

Color ray_color(const Ray& r, const Hittable& world,
                const shared_ptr<HittableList>& lights, int depth) {
  HitRecord rec;
  if (depth <= 0) return Color(0, 0, 0);

  if (!world.hit(r, 0.001, infinity, rec)) return Color(0, 0, 0);

  ScatterRecord srec;
  Color emitted = rec.mat_ptr->emitted(r, rec, 0, 0, rec.p);

  if (!rec.mat_ptr->scatter(r, rec, srec)) return emitted;

  if (srec.is_specular) {
    return srec.attenuation *
           ray_color(srec.specular_ray, world, lights, depth - 1);
  }

  auto light_ptr = make_shared<HittablePdf>(lights, rec.p);
  MixturePdf p(light_ptr, srec.pdf_ptr);

  Ray scattered = Ray(rec.p, p.generate());
  auto pdf_val = p.value(scattered.direction());

  if (pdf_val == 0) return emitted;

  return emitted + srec.attenuation *
                       rec.mat_ptr->scattering_pdf(r, rec, scattered) *
                       ray_color(scattered, world, lights, depth - 1) / pdf_val;
}

void add_box(HittableList& world, Point3 p0, Point3 p1,
             shared_ptr<Material> mat) {
  Point3 min_p(fmin(p0.x(), p1.x()), fmin(p0.y(), p1.y()),
               fmin(p0.z(), p1.z()));
  Point3 max_p(fmax(p0.x(), p1.x()), fmax(p0.y(), p1.y()),
               fmax(p0.z(), p1.z()));

  Vec3 dx(max_p.x() - min_p.x(), 0, 0);
  Vec3 dy(0, max_p.y() - min_p.y(), 0);
  Vec3 dz(0, 0, max_p.z() - min_p.z());

  world.add(
      make_shared<Quad>(Point3(min_p.x(), min_p.y(), max_p.z()), dx, dy, mat));
  world.add(
      make_shared<Quad>(Point3(max_p.x(), min_p.y(), max_p.z()), -dz, dy, mat));
  world.add(
      make_shared<Quad>(Point3(max_p.x(), min_p.y(), min_p.z()), -dx, dy, mat));
  world.add(
      make_shared<Quad>(Point3(min_p.x(), min_p.y(), min_p.z()), dz, dy, mat));

  world.add(
      make_shared<Quad>(Point3(min_p.x(), max_p.y(), min_p.z()), dz, dx, mat));
}

int main() {
  const auto aspect_ratio = 1.0;
  const int image_width = 400;
  const int image_height = static_cast<int>(image_width / aspect_ratio);
  const int samples_per_pixel = 200;
  const int max_depth = 20;

  HittableList world;
  auto lights = make_shared<HittableList>();

  auto mat_reflective_dark = make_shared<PrincipledBSDF>(
      Color(0.002, 0.002, 0.002), 0.3, 0.1, 0.0, 1.5);

  auto mat_light_white = make_shared<DiffuseLight>(Color(400, 400, 400));
  auto mat_light_purple = make_shared<DiffuseLight>(Color(80.0, 0.0, 80.0));
  auto mat_light_cyan = make_shared<DiffuseLight>(Color(0.0, 80.0, 80.0));

  auto mat_sapphire = make_shared<PrincipledBSDF>(
      Color(1.0, 1.0, 1.0), 0.0, 0.0, 1.0, 1.76, Color(0.02, 0.02, 0.001));

  auto mat_gold_bright =
      make_shared<PrincipledBSDF>(Color(0.95, 0.7, 0.3), 1.0, 0.3, 0.0, 1.5);

  world.add(make_shared<Quad>(Point3(555, 0, 0), Vec3(0, 555, 0),
                              Vec3(0, 0, 555), mat_reflective_dark));
  world.add(make_shared<Quad>(Point3(0, 0, 0), Vec3(0, 555, 0), Vec3(0, 0, 555),
                              mat_reflective_dark));
  world.add(make_shared<Quad>(Point3(0, 0, 0), Vec3(555, 0, 0), Vec3(0, 0, 555),
                              mat_reflective_dark));
  world.add(make_shared<Quad>(Point3(0, 555, 0), Vec3(555, 0, 0),
                              Vec3(0, 0, 555), mat_reflective_dark));
  world.add(make_shared<Quad>(Point3(0, 0, 555), Vec3(555, 0, 0),
                              Vec3(0, 555, 0), mat_reflective_dark));

  double light_size = 200.0;
  double offset = (555.0 - light_size) / 2.0;
  auto ceiling_light = make_shared<Quad>(
      Point3(offset, 554, offset + 50), Vec3(light_size, 0, 0),
      Vec3(0, 0, light_size), mat_light_white);
  world.add(ceiling_light);
  lights->add(ceiling_light);

  double center_x = 555.0 / 2.0;

  double place_z = 100.0;

  double table_w = 220;
  double table_h = 200;

  double table_x0 = center_x - table_w / 2;
  double table_z0 = place_z - table_w / 2;

  add_box(world, Point3(table_x0, 0, table_z0),
          Point3(table_x0 + table_w, table_h, table_z0 + table_w),
          mat_gold_bright);

  auto bunny = load_obj("assets/bunny.obj", Point3(center_x, table_h, place_z),
                        1000.0, 180.0, mat_sapphire);
  world.add(bunny);

  double tube_r = 10.0;
  double wall_gap = 5.0;
  world.add(make_shared<Cylinder>(
      Point3(555 - tube_r - wall_gap, 0, 555 - tube_r - wall_gap), 555, tube_r,
      mat_light_purple));
  world.add(make_shared<Cylinder>(
      Point3(0 + tube_r + wall_gap, 0, 555 - tube_r - wall_gap), 555, tube_r,
      mat_light_cyan));

  std::cerr << "Building SAH BVH...\n";
  auto bvh_root = make_shared<BvhNode>(world, 0, 1);
  std::cerr << "Rendering...\n";

  Point3 lookfrom(center_x, 300, -500);

  Point3 lookat(center_x, 300, place_z);

  Vec3 vup(0, 1, 0);
  auto dist_to_focus = (lookfrom - lookat).length();

  auto theta = degrees_to_radians(40.0);
  auto h = tan(theta / 2);
  auto viewport_height = 2.0 * h;
  auto viewport_width = aspect_ratio * viewport_height;

  auto w = unit_vector(lookfrom - lookat);
  auto u = unit_vector(cross(vup, w));
  auto v = cross(w, u);

  auto origin = lookfrom;
  auto horizontal = dist_to_focus * viewport_width * u;
  auto vertical = dist_to_focus * viewport_height * v;
  auto lower_left_corner =
      origin - horizontal / 2 - vertical / 2 - dist_to_focus * w;

  std::cout << "P3\n" << image_width << ' ' << image_height << "\n255\n";

  std::vector<Color> buffer(image_width * image_height);
  std::atomic<int> completed_lines(0);

#pragma omp parallel for schedule(dynamic, 1)
  for (int j = image_height - 1; j >= 0; --j) {
    for (int i = 0; i < image_width; ++i) {
      Color pixel_color(0, 0, 0);
      for (int s = 0; s < samples_per_pixel; ++s) {
        auto u_offset = random_double();
        auto v_offset = random_double();
        auto u_coord = (i + u_offset) / (image_width - 1);
        auto v_coord = (j + v_offset) / (image_height - 1);

        Ray r(origin, lower_left_corner + u_coord * horizontal +
                          v_coord * vertical - origin);
        pixel_color += ray_color(r, *bvh_root, lights, max_depth);
      }
      buffer[(image_height - 1 - j) * image_width + i] = pixel_color;
    }
    int finished = ++completed_lines;
    std::cerr << "\rProgress: " << int(100.0 * finished / image_height) << "% "
              << std::flush;
  }

  for (const auto& pixel_color : buffer) {
    auto scale = 1.0 / samples_per_pixel;
    Color col = pixel_color * scale;
    if (col.x() != col.x()) col = Color(0, 0, 0);
    col = aces_approx(col);
    auto r = pow(col.x(), 1.0 / 2.2);
    auto g = pow(col.y(), 1.0 / 2.2);
    auto b = pow(col.z(), 1.0 / 2.2);
    std::cout << static_cast<int>(256 * clamp(r, 0.0, 0.999)) << ' '
              << static_cast<int>(256 * clamp(g, 0.0, 0.999)) << ' '
              << static_cast<int>(256 * clamp(b, 0.0, 0.999)) << '\n';
  }
  std::cerr << "\nDone.\n";
}