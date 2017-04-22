#include "environment_light.h"

#include <algorithm>
#include <iostream>
#include <fstream>

namespace CGL { namespace StaticScene {

EnvironmentLight::EnvironmentLight(const HDRImageBuffer* envMap)
    : envMap(envMap) {
    	init();
}

EnvironmentLight::~EnvironmentLight() {
    delete[] pdf_envmap;
    delete[] conds_y;
    delete[] marginal_y;
}


void EnvironmentLight::init() {
	uint32_t w = envMap->w, h = envMap->h;
    pdf_envmap = new double[w * h];
	conds_y = new double[w * h];
	marginal_y = new double[h];

	std::cout << "[PathTracer] Initializing environment light...";

	// TODO 3-2 Part 3 Task 3 Steps 1,2,3
	// Store the environment map pdf to pdf_envmap
	// Store the marginal distribution for y to marginal_y
	// Store the conditional distribution for x given y to conds_y

	// Store constant denom
	double denom = 0.0;
	for (int j = 0; j < (int) h; j++) {
		for (int i = 0; i < (int) w; i++) {
			denom += envMap->data[j * w + i].illum() * sin(PI * j / h);
		}
	}

	// Calculate pdf
	for (int y = 0; y < (int) h; y++) {
		for (int x = 0; x < (int) w; x++) {
			pdf_envmap[y*w + x] += (envMap->data[y * w + x].illum() * sin(PI * (double) (y) / h)) / denom;
		}
	}

	// Store marginal y's distribution
	double marginal_density_y[h];
	for (int j = 0; j < (int) h; j++) {
		marginal_y[j] = 0.0;
		marginal_density_y[j] = 0.0;
		if (j != 0) {
			marginal_y[j] = marginal_y[j - 1];
		}
		for (int i = 0; i < (int) w; i++) {
			marginal_density_y[j] += pdf_envmap[j*w + i];
			marginal_y[j] += pdf_envmap[j*w + i];
		}
	}

	// conditional probs
	for (int j = 0; j < (int) h; j++) {
		for (int i = 0; i < (int) w; i++) {
			double newTerm = (pdf_envmap[j*w + i] / marginal_density_y[j]);
			if (i != 0) {
				conds_y[j*w + i] += (conds_y[j*w + (i - 1)] + newTerm);
			} else {
				conds_y[j*w + i] += newTerm;
			}
		}
	}

	if (false)
		save_probability_debug();

	std::cout << "done." << std::endl;
}


// Helper functions

void EnvironmentLight::save_probability_debug() {
	uint32_t w = envMap->w, h = envMap->h;
	uint8_t* img = new uint8_t[4*w*h];

	for (int j = 0; j < h; ++j) {
		for (int i = 0; i < w; ++i) {
			img[4 * (j * w + i) + 3] = 255;
			img[4 * (j * w + i) + 0] = 255 * marginal_y[j];
			img[4 * (j * w + i) + 1] = 255 * conds_y[j * w + i];
		}
	}

    lodepng::encode("probability_debug.png", img, w, h);
    delete[] img;
}

Vector2D EnvironmentLight::theta_phi_to_xy(const Vector2D &theta_phi) const {
    uint32_t w = envMap->w, h = envMap->h;
    double x = theta_phi.y / 2. / M_PI * w;
    double y = theta_phi.x / M_PI * h;
    return Vector2D(x, y);
}

Vector2D EnvironmentLight::xy_to_theta_phi(const Vector2D &xy) const {
    uint32_t w = envMap->w, h = envMap->h;
    double x = xy.x;
    double y = xy.y;
    double phi = x / w * 2.0 * M_PI;
    double theta = y / h * M_PI;
    return Vector2D(theta, phi);
}

Vector2D EnvironmentLight::dir_to_theta_phi(const Vector3D &dir) const {
    dir.unit();
    double theta = acos(dir.y);
    double phi = atan2(-dir.z, dir.x) + M_PI;
    return Vector2D(theta, phi);
}

Vector3D EnvironmentLight::theta_phi_to_dir(const Vector2D& theta_phi) const {
    double theta = theta_phi.x;
    double phi = theta_phi.y;

    double y = cos(theta);
    double x = cos(phi - M_PI) * sin(theta);
    double z = -sin(phi - M_PI) * sin(theta);

    return Vector3D(x, y, z);
}

Spectrum EnvironmentLight::bilerp(const Vector2D& xy) const {
	uint32_t w = envMap->w;
	const std::vector<Spectrum>& data = envMap->data;
	double x = xy.x, y = xy.y;
	Spectrum ret;
	for (int i = 0; i < 4; ++i)
		ret += (i%2 ? x-floor(x) : ceil(x)-x) * 
			   (i/2 ? y-floor(y) : ceil(y)-y) * 
			   data[w * (floor(y) + i/2) + floor(x) + i%2];
	return ret;
}


Spectrum EnvironmentLight::sample_L(const Vector3D& p, Vector3D* wi,
                                    float* distToLight,
                                    float* pdf) const {
  	// TODO: 3-2 Part 3 Tasks 2 and 3 (step 4)
	// First implement uniform sphere sampling for the environment light
	// Later implement full importance sampling

	// Uniform sphere sampling
	// Vector3D sample = sampler_uniform_sphere.get_sample();
	// Vector2D theta_phi = dir_to_theta_phi(sample);

	// *wi = sample;
	// *distToLight = INF_D;
	// *pdf = 1.0 / (4.0 * PI);

	// return bilerp(theta_phi_to_xy(theta_phi));

	// Importance sampling
	uint32_t w = envMap->w, h = envMap->h;

	Vector2D sample = sampler_uniform2d.get_sample();
	int newX, newY;
	newY = std::upper_bound(marginal_y, &(marginal_y[h - 1]), sample.y) - marginal_y;
	newX = std::upper_bound(&(conds_y[newY * w]), &(conds_y[(newY + 1) * w]), sample.x) - &(conds_y[newY * w]);
	Vector2D xy = Vector2D(newX, newY);
	Vector3D dir = theta_phi_to_dir(xy_to_theta_phi(xy));
	*wi = dir;
	*distToLight = INF_D;
	*pdf = pdf_envmap[newY*w + newX] * w * h / (2.0 * PI * PI * sin_theta(dir));

	return envMap->data[newY*w + newX];
}

Spectrum EnvironmentLight::sample_dir(const Ray& r) const {
  // TODO: 3-2 Part 3 Task 1
	// Use the helper functions to convert r.d into (x,y)
	// then bilerp the return value 
	Vector2D theta_phi = dir_to_theta_phi(r.d);
	Vector2D xy = theta_phi_to_xy(theta_phi);

	return bilerp(xy);
}

} // namespace StaticScene
} // namespace CGL
