#include "bsdf.h"

#include <iostream>
#include <algorithm>
#include <utility>

using std::min;
using std::max;
using std::swap;

namespace CGL {

void make_coord_space(Matrix3x3& o2w, const Vector3D& n) {

    Vector3D z = Vector3D(n.x, n.y, n.z);
    Vector3D h = z;
    if (fabs(h.x) <= fabs(h.y) && fabs(h.x) <= fabs(h.z)) h.x = 1.0;
    else if (fabs(h.y) <= fabs(h.x) && fabs(h.y) <= fabs(h.z)) h.y = 1.0;
    else h.z = 1.0;

    z.normalize();
    Vector3D y = cross(h, z);
    y.normalize();
    Vector3D x = cross(z, y);
    x.normalize();

    o2w[0] = x;
    o2w[1] = y;
    o2w[2] = z;
}


// Diffuse BSDF //

Spectrum DiffuseBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return this->reflectance / PI;
}

Spectrum DiffuseBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  *wi = this->sampler.get_sample(pdf);
  return this->reflectance / PI;
}


// Mirror BSDF //

Spectrum MirrorBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum MirrorBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {

  // TODO: 3-2 Part 1 Task 2
  // Implement MirrorBSDF

  *pdf = 1.0;
  BSDF::reflect(wo, wi);

  return reflectance / abs_cos_theta(*wi);
}


// Microfacet BSDF //

Spectrum MicrofacetBSDF::F(const Vector3D& wi) {
  // TODO: proj3-2, part 2
  // Compute Fresnel term for reflection on dielectric-conductor interface.
  // You will need both eta and etaK, both of which are Spectrum.
  Spectrum Rs = ((eta*eta + k*k) - (2.0 * abs_cos_theta(wi) * eta) + (abs_cos_theta(wi)*abs_cos_theta(wi))) / 
              ((eta*eta + k*k) + (2.0 * abs_cos_theta(wi) * eta) + (abs_cos_theta(wi)*abs_cos_theta(wi)));
  Spectrum Rp = ((eta*eta + k*k) * (abs_cos_theta(wi) * abs_cos_theta(wi)) - (2.0 * eta * abs_cos_theta(wi)) + 1.0) / 
              ((eta*eta + k*k) * (abs_cos_theta(wi) * abs_cos_theta(wi)) + (2.0 * eta * abs_cos_theta(wi)) + 1.0);
  // return Spectrum(1.0, 1.0, 1.0);
  return (Rs + Rp) / 2.0;
}

double MicrofacetBSDF::G(const Vector3D& wo, const Vector3D& wi) {
    return 1.0 / (1.0 + Lambda(wi) + Lambda(wo));
}

double MicrofacetBSDF::D(const Vector3D& h) {
  // TODO: proj3-2, part 2
  // Compute Beckmann normal distribution function (NDF) here.
  // You will need the roughness alpha.
  double alpha = 0.25;
  Vector3D normal = Vector3D(0.0, 0.0, 1.0);

  double cost_theta_h = cos_theta(h);

  // return std::pow(cos_theta(h), 100.0);
  return exp(-1.0 * pow(sin_theta(h) / cos_theta(h), 2.0) / (alpha * alpha)) / (PI * alpha * alpha * pow(cos_theta(h), 4.0));
}

Spectrum MicrofacetBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  // TODO: proj3-2, part 2
  // Implement microfacet model here.
  if (wo.z <= 0.0 || wi.z <= 0.0) {
    return Spectrum();
  }
  Vector3D h = (wo + wi) / (wo + wi).norm();
  Vector3D normal = Vector3D(0.0, 0.0, 1.0);
  return (F(wi) * G(wo, wi) * D(h)) / (4.0 * dot(normal, wo) * dot(normal, wi)) ;
}

Spectrum MicrofacetBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  // TODO: proj3-2, part 2
  // *Importance* sample Beckmann normal distribution function (NDF) here.
  // Note: You should fill in the sampled direction *wi and the corresponding *pdf,
  //       and return the sampled BRDF value.

  // *wi = cosineHemisphereSampler.get_sample(pdf);
  // return MicrofacetBSDF::f(wo, *wi);

  Vector2D r = sampler.get_sample();

  double theta_h = atan(sqrt(-1.0 * alpha * alpha * log(1.0 - r.x)));
  double phi_h = 2.0 * PI * r.y;

  double p_theta_h = ( (2.0 * sin(theta_h)) / (alpha * alpha * pow(cos(theta_h), 3.0))) *
                      exp( -1.0 * pow(sin(theta_h) / cos(theta_h), 2.0) / (alpha * alpha));
  double p_phi_h = 1.0 / (2.0 * PI);

  double p_h = p_theta_h * p_phi_h / sin(theta_h);
  Vector3D h = Vector3D(sin(theta_h) * cos(phi_h), sin(phi_h) * sin(theta_h), cos(theta_h));

  *wi = -1.0 * wo + 2.0 * dot(wo, h) * h;

  if (wi->z <= 0.0) {
    *pdf = 0.0;
    return Spectrum();
  }

  *pdf = p_h / (4.0 * dot(*wi, h));

  return f(wo, *wi);
}


// Refraction BSDF //

Spectrum RefractionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum RefractionBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {

  
  return Spectrum();
}

// Glass BSDF //

Spectrum GlassBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum GlassBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {

  // TODO: 3-2 Part 1 Task 4
  // Compute Fresnel coefficient and either reflect or refract based on it.

  double eta;

  if (wo.z < 0.0) {
    eta = ior;
  } else {
    eta = 1.0 / ior;
  }

  if ( (1.0 - pow(eta,2.0) * (1.0 - pow(abs_cos_theta(wo), 2.0))) < 0.0) {
    BSDF::reflect(wo, wi);
    *pdf = 1.0;
    return reflectance / abs_cos_theta(*wi);
  }
  else {
    double n1 = 1.0;
    double n2 = ior;

    double r0 = pow((n1 - n2) / (n1 + n2), 2.0);
    double r = r0 + (1.0 - r0) * pow((1.0 - abs_cos_theta(wo)), 5.0);

    if (coin_flip(r)) {
      BSDF::reflect(wo, wi);
      *pdf = r;
      return r * reflectance / abs_cos_theta(*wi);
    } else {
      BSDF::refract(wo, wi, ior);
      *pdf = 1.0 - r;
      return (1.0 - r) * transmittance / abs_cos_theta(*wi) / pow(eta, 2.0);
    }
  }
  
  return Spectrum();
}

void BSDF::reflect(const Vector3D& wo, Vector3D* wi) {

  // TODO: 3-2 Part 1 Task 1
  // Implement reflection of wo about normal (0,0,1) and store result in wi.
  *wi = Vector3D(-wo[0],-wo[1],wo[2]);

}

bool BSDF::refract(const Vector3D& wo, Vector3D* wi, float ior) {

  // TODO: 3-2 Part 1 Task 3
  // Use Snell's Law to refract wo surface and store result ray in wi.
  // Return false if refraction does not occur due to total internal reflection
  // and true otherwise. When wo.z is positive, then wo corresponds to a
  // ray entering the surface through vacuum.


  double wo_x = wo.x;
  double wo_y = wo.y;
  double wo_z = wo.z;

  double n;
  bool entering = true;

  if (wo_z < 0.0) {
    n = ior;
    entering = false;
  } else {
    n = 1.0 / ior;
  }

  if (1.0 - pow(n,2.0) * (1.0 - pow(wo_z, 2.0)) < 0.0) {
    return false;
  }

  if (wo_z >= 0.0) {
    *wi = Vector3D(-1.0 * n * wo_x, -1.0 * n * wo_y, -1.0 * sqrt(1.0 - pow(n,2.0) * (1.0 - pow(wo_z, 2.0))));
  } else {
    *wi = Vector3D(-1.0 * n * wo_x, -1.0 * n * wo_y, sqrt(1.0 - pow(n,2.0) * (1.0 - pow(wo_z, 2.0))));
  }

  return true;

}

// Emission BSDF //

Spectrum EmissionBSDF::f(const Vector3D& wo, const Vector3D& wi) {
  return Spectrum();
}

Spectrum EmissionBSDF::sample_f(const Vector3D& wo, Vector3D* wi, float* pdf) {
  *pdf = 1.0 / PI;
  *wi  = sampler.get_sample(pdf);
  return Spectrum();
}

} // namespace CGL
