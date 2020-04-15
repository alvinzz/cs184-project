#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
  delete gridSampler;
  delete hemisphereSampler;
}

void PathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
}

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                      size_t y0, size_t x1, size_t y1) {
  sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Spectrum
PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // For this function, sample uniformly in a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D &hit_p = r.o + r.d * isect.t;
  const Vector3D &w_out = w2o * (-r.d);

  // This is the same number of total samples as
  // estimate_direct_lighting_importance (outside of delta lights). We keep the
  // same number of samples for clarity of comparison.
  int num_samples = scene->lights.size() * ns_area_light;
  Spectrum L_out;

  // TODO (Part 3): Write your sampling loop here
  // TODO BEFORE YOU BEGIN
  // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading 
  Vector3D wi_o;
  for (int sample = 0; sample < num_samples; sample++) {
    Vector3D wi_o = hemisphereSampler->get_sample();
    Ray ri = Ray(hit_p, o2w*wi_o);
    ri.min_t = EPS_D;
    ri.max_t = INFINITY;
    Intersection isect2 = Intersection();
    if (bvh->intersect(ri, &isect2)) {
      Spectrum Li = isect2.bsdf->get_emission();
      Spectrum emission = isect.bsdf->f(w_out, wi_o);
      L_out += wi_o[2] * Li * emission * 2*PI / (float)num_samples;
    }
  }

  return L_out;
}

Spectrum
PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // To implement importance sampling, sample only from lights, not uniformly in
  // a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D &hit_p = r.o + r.d * isect.t;
  const Vector3D &w_out = w2o * (-r.d);
  Spectrum L_out;

  for (SceneLight *light : scene->lights) {
    if (light->is_delta_light()) {
      Vector3D wi_w;
      float dist;
      float pdf;
      Spectrum Li = light->sample_L(hit_p, &wi_w, &dist, &pdf);
      if ((w2o * wi_w)[2] > 0) {
        Ray ri = Ray(hit_p, wi_w);
        ri.min_t = EPS_D;
        ri.max_t = dist - EPS_D;
        if (!(bvh->has_intersection(ri, bvh->get_root()))) {
          Spectrum emission = isect.bsdf->f(w_out, w2o * wi_w);
          L_out += (w2o * wi_w)[2] * Li * emission / pdf;
        }
      }
    } else {
      Vector3D wi_w;
      float dist;
      float pdf;
      for (int sample = 0; sample < ns_area_light; sample++) {
        Spectrum Li = light->sample_L(hit_p, &wi_w, &dist, &pdf);
        if ((w2o * wi_w)[2] > 0) {
          Ray ri = Ray(hit_p, wi_w);
          ri.min_t = EPS_D;
          ri.max_t = dist - EPS_D;
          if (!(bvh->has_intersection(ri, bvh->get_root()))) {
            Spectrum emission = isect.bsdf->f(w_out, w2o * wi_w);  
            L_out += (w2o * wi_w)[2] * Li * emission / pdf / (float)ns_area_light;
          }
        }
      }
    }
  }

  return L_out;
}

Spectrum PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  // TODO: Part 3, Task 2
  // Returns the light that results from no bounces of light

  return isect.bsdf->get_emission();
}

Spectrum PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  // TODO: Part 3, Task 3
  // Returns either the direct illumination by hemisphere or importance sampling
  // depending on `direct_hemisphere_sample`

  // return estimate_direct_lighting_hemisphere(r, isect);
  return estimate_direct_lighting_importance(r, isect);
}

Spectrum PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  Vector3D hit_p = r.o + r.d * isect.t;
  Vector3D w_out = w2o * (-r.d);

  Spectrum L_out;
  if (r.depth == 0) {
    L_out = one_bounce_radiance(r, isect);
    // L_out = Spectrum(0., 0., 0.);
  } else {
    L_out = one_bounce_radiance(r, isect);
  }
  if (r.depth == max_ray_depth) {
    return L_out;
  }

  double continue_prob = 0.666;
  if (coin_flip(continue_prob)) {
    Vector3D wi_o;
    float pdf;
    Spectrum emission = isect.bsdf->sample_f(w_out, &wi_o, &pdf);
    Intersection isect2;
    Ray r2 = Ray(hit_p, o2w * wi_o);
    r2.depth = r.depth + 1;
    r2.min_t = EPS_D;
    r2.max_t = INFINITY;
    if (bvh->intersect(r2, &isect2)) {
      L_out += at_least_one_bounce_radiance(r2, isect2) * emission * wi_o[2] / pdf / continue_prob;
    }
  }

  return L_out;
}

Spectrum PathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Spectrum L_out;

  // You will extend this in assignment 3-2.
  // If no intersection occurs, we simply return black.
  // This changes if you implement hemispherical lighting for extra credit.

  if (!bvh->intersect(r, &isect)) {
    return L_out;    
  }


  // The following line of code returns a debug color depending
  // on whether ray intersection with triangles or spheres has
  // been implemented.

  // REMOVE THIS LINE when you are ready to begin Part 3.
  // L_out = (isect.t == INF_D) ? (Spectrum)Vector3D(abs(r.d.r), abs(r.d.g), .0).unit() : normal_shading(isect.n);
  // return L_out;

  // TODO (Part 3): Return the direct illumination.
  // return zero_bounce_radiance(r, isect) + one_bounce_radiance(r, isect);

  // TODO (Part 4): Accumulate the "direct" and "indirect"
  // parts of global illumination into L_out rather than just direct
  return zero_bounce_radiance(r, isect) + at_least_one_bounce_radiance(r, isect);
}

void PathTracer::raytrace_pixel(size_t x, size_t y) {

  // TODO (Part 1.1):
  // Make a loop that generates num_samples camera rays and traces them
  // through the scene. Return the average Spectrum.
  // You should call est_radiance_global_illumination in this function.

  // Spectrum px_val = Spectrum(0.0);
  // for (int sample = 0; sample < ns_aa; sample++) {
  //   Ray ray = camera->generate_ray(
  //     (x+random_uniform()) / sampleBuffer.w,
  //     (y+random_uniform()) / sampleBuffer.h
  //   );
  //   px_val += est_radiance_global_illumination(ray) / ns_aa;
  // }
  // sampleBuffer.update_pixel(px_val, x, y);


  // TODO (Part 5):
  // Modify your implementation to include adaptive sampling.
  // Use the command line parameters "samplesPerBatch" and "maxTolerance"

  int num_samples = 0;          // total samples to evaluate
  int sample;
  double maxTolerance = 0.05;
  double mu;
  double sigma;
  double I;
  double s1 = 0;
  double s2 = 0;
  Spectrum px_val = Spectrum(0.0);
  for (sample = 1; sample <= ns_aa; sample++) {
    Ray ray = camera->generate_ray(
      (x+random_uniform()) / sampleBuffer.w,
      (y+random_uniform()) / sampleBuffer.h
    );
    Spectrum sample_spectrum = est_radiance_global_illumination(ray);
    px_val += sample_spectrum;
    s1 += sample_spectrum.illum();
    s2 += sample_spectrum.illum() * sample_spectrum.illum();
    if (sample % samplesPerBatch == 0) {
      mu = s1 / sample;
      sigma = (sample > 1) ? sqrt((s2 - s1*s1 / sample) / (sample - 1)) : INFINITY;
      I = (sample > 1) ? (1.96 * sigma / sqrt(sample)) : INFINITY;
      if (I <= maxTolerance * mu) {
        num_samples = sample;
        break;
      }
    }
  }
  if (sample == ns_aa + 1) {
    num_samples = ns_aa;
  }
  px_val /= num_samples;
  sampleBuffer.update_pixel(px_val, x, y);
  sampleCountBuffer[x + y * sampleBuffer.w] = num_samples;
}

} // namespace CGL