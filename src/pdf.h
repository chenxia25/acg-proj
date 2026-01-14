#ifndef PDF_H
#define PDF_H

#include "hittable.h"
#include "onb.h"
#include "utility.h"

struct Pdf {
  virtual ~Pdf() {}
  virtual double value(const Vec3& direction) const = 0;
  virtual Vec3 generate() const = 0;
};

struct CosinePdf : public Pdf {
  Onb uvw;

  CosinePdf(const Vec3& w) { uvw.build_from_w(w); }

  virtual double value(const Vec3& direction) const override {
    auto cosine = dot(unit_vector(direction), uvw.w());
    return (cosine <= 0) ? 0 : cosine / pi;
  }

  virtual Vec3 generate() const override {
    auto r1 = random_double();
    auto r2 = random_double();
    auto z = sqrt(1 - r2);
    auto phi = 2 * pi * r1;
    auto x = cos(phi) * sqrt(r2);
    auto y = sin(phi) * sqrt(r2);
    return uvw.local(x, y, z);
  }
};

struct HittablePdf : public Pdf {
  Point3 o;
  shared_ptr<Hittable> ptr;

  HittablePdf(shared_ptr<Hittable> p, const Point3& origin)
      : ptr(p), o(origin) {}

  virtual double value(const Vec3& direction) const override {
    return ptr->pdf_value(o, direction);
  }

  virtual Vec3 generate() const override { return ptr->random(o); }
};

struct MixturePdf : public Pdf {
  shared_ptr<Pdf> p[2];

  MixturePdf(shared_ptr<Pdf> p0, shared_ptr<Pdf> p1) {
    p[0] = p0;
    p[1] = p1;
  }

  virtual double value(const Vec3& direction) const override {
    return 0.5 * p[0]->value(direction) + 0.5 * p[1]->value(direction);
  }

  virtual Vec3 generate() const override {
    if (random_double() < 0.5)
      return p[0]->generate();
    else
      return p[1]->generate();
  }
};

#endif