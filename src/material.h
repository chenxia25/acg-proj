#ifndef MATERIAL_H
#define MATERIAL_H

#include "hittable.h"
#include "pdf.h"
#include "utility.h"

struct HitRecord;

struct ScatterRecord {
  Ray specular_ray;
  bool is_specular;
  Color attenuation;
  shared_ptr<Pdf> pdf_ptr;
};

struct Material {
  virtual bool scatter(const Ray& r_in, const HitRecord& rec,
                       ScatterRecord& srec) const {
    return false;
  }

  virtual double scattering_pdf(const Ray& r_in, const HitRecord& rec,
                                const Ray& scattered) const {
    return 0;
  }

  virtual Color emitted(const Ray& r_in, const HitRecord& rec, double u,
                        double v, const Point3& p) const {
    return Color(0, 0, 0);
  }
};

struct DiffuseLight : public Material {
  Color emit;
  DiffuseLight(Color c) : emit(c) {}

  virtual bool scatter(const Ray& r_in, const HitRecord& rec,
                       ScatterRecord& srec) const override {
    return false;
  }

  virtual Color emitted(const Ray& r_in, const HitRecord& rec, double u,
                        double v, const Point3& p) const override {
    return emit;
  }
};
struct Lambertian : public Material {
  Color albedo;
  Lambertian(const Color& a) : albedo(a) {}

  virtual bool scatter(const Ray& r_in, const HitRecord& rec,
                       ScatterRecord& srec) const override {
    srec.is_specular = false;
    srec.attenuation = albedo;
    srec.pdf_ptr = make_shared<CosinePdf>(rec.normal);
    return true;
  }

  virtual double scattering_pdf(const Ray& r_in, const HitRecord& rec,
                                const Ray& scattered) const override {
    auto cosine = dot(rec.normal, unit_vector(scattered.direction()));
    return cosine < 0 ? 0 : cosine / pi;
  }
};

struct PrincipledBSDF : public Material {
  Color albedo;
  double metallic;
  double roughness;
  double transmission;
  double ior;
  Color absorption;
  double normal_strength;

  PrincipledBSDF(Color a, double m, double r, double t, double index,
                 Color abs = Color(0, 0, 0), double ns = 0.0)
      : albedo(a),
        metallic(m),
        roughness(r),
        transmission(t),
        ior(index),
        absorption(abs),
        normal_strength(ns) {}

  virtual bool scatter(const Ray& r_in, const HitRecord& original_rec,
                       ScatterRecord& srec) const override {
    HitRecord rec = original_rec;
    if (normal_strength > 0.0) {
      double freq = 20.0;
      Vec3 perturbation(sin(rec.p.x() * freq), sin(rec.p.y() * freq),
                        sin(rec.p.z() * freq));
      rec.normal = unit_vector(rec.normal + perturbation * normal_strength);
    }

    bool is_specular_bounce =
        (random_double() < metallic) || (random_double() < transmission);

    if (is_specular_bounce) {
      srec.is_specular = true;
      srec.pdf_ptr = nullptr;

      double refraction_ratio = rec.front_face ? (1.0 / ior) : ior;
      Vec3 unit_direction = unit_vector(r_in.direction());
      double cos_theta = fmin(dot(-unit_direction, rec.normal), 1.0);
      double sin_theta = sqrt(1.0 - cos_theta * cos_theta);

      bool cannot_refract = refraction_ratio * sin_theta > 1.0;
      bool should_reflect =
          cannot_refract ||
          reflectance(cos_theta, refraction_ratio) > random_double();
      if (metallic > 0.5) should_reflect = true;

      Vec3 target_dir;
      if (should_reflect) {
        target_dir = reflect(unit_direction, rec.normal);
        srec.attenuation = albedo;
      } else {
        double dispersion_strength = 0.05;
        double jittered_ior = ior;
        Color tint = Color(1, 1, 1);

        double choice = random_double();
        if (choice < 0.33) {
          jittered_ior -= dispersion_strength;
          tint = Color(1, 0.2, 0.2);
        } else if (choice < 0.66) {
          jittered_ior += 0.0;
          tint = Color(0.2, 1, 0.2);
        } else {
          jittered_ior += dispersion_strength;
          tint = Color(0.2, 0.2, 1);
        }

        double ratio = rec.front_face ? (1.0 / jittered_ior) : jittered_ior;
        target_dir = refract(unit_direction, rec.normal, ratio);

        srec.attenuation = tint * 3.0;
      }

      if (!rec.front_face) {
        Color absorb_factor(exp(-absorption.x() * rec.t),
                            exp(-absorption.y() * rec.t),
                            exp(-absorption.z() * rec.t));
        srec.attenuation = srec.attenuation * absorb_factor;
      }

      if (roughness > 0) target_dir += roughness * random_in_unit_sphere();

      srec.specular_ray = Ray(rec.p, target_dir);
      return true;
    }

    srec.is_specular = false;
    srec.attenuation = albedo;
    srec.pdf_ptr = make_shared<CosinePdf>(rec.normal);
    return true;
  }

  virtual double scattering_pdf(const Ray& r_in, const HitRecord& rec,
                                const Ray& scattered) const override {
    auto cosine = dot(rec.normal, unit_vector(scattered.direction()));
    return cosine < 0 ? 0 : cosine / pi;
  }
};

#endif