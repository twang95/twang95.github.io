#include "camera.h"

#include <iostream>
#include <sstream>
#include <fstream>

#include "CGL/misc.h"
#include "CGL/vector2D.h"
#include "CGL/vector3D.h"

using std::cout;
using std::endl;
using std::max;
using std::min;
using std::ifstream;
using std::ofstream;

namespace CGL {

using Collada::CameraInfo;

void Camera::configure(const CameraInfo& info, size_t screenW, size_t screenH) {
  this->screenW = screenW;
  this->screenH = screenH;
  nClip = info.nClip;
  fClip = info.fClip;
  hFov = info.hFov;
  vFov = info.vFov;

  double ar1 = tan(radians(hFov) / 2) / tan(radians(vFov) / 2);
  ar = static_cast<double>(screenW) / screenH;
  if (ar1 < ar) {
    // hFov is too small
    hFov = 2 * degrees(atan(tan(radians(vFov) / 2) * ar));
  } else if (ar1 > ar) {
    // vFov is too small
    vFov = 2 * degrees(atan(tan(radians(hFov) / 2) / ar));
  }
  screenDist = ((double) screenH) / (2.0 * tan(radians(vFov) / 2));

}

void Camera::place(const Vector3D& targetPos, const double phi,
                   const double theta, const double r, const double minR,
                   const double maxR) {
  double r_ = min(max(r, minR), maxR);
  double phi_ = (sin(phi) == 0) ? (phi + EPS_F) : phi;
  this->targetPos = targetPos;
  this->phi = phi_;
  this->theta = theta;
  this->r = r_;
  this->minR = minR;
  this->maxR = maxR;
  compute_position();
}

void Camera::copy_placement(const Camera& other) {
  pos = other.pos;
  targetPos = other.targetPos;
  phi = other.phi;
  theta = other.theta;
  minR = other.minR;
  maxR = other.maxR;
  c2w = other.c2w;
}

void Camera::set_screen_size(const size_t screenW, const size_t screenH) {
  this->screenW = screenW;
  this->screenH = screenH;
  ar = 1.0 * screenW / screenH;
  hFov = 2 * degrees(atan(((double) screenW) / (2 * screenDist)));
  vFov = 2 * degrees(atan(((double) screenH) / (2 * screenDist)));
}

void Camera::move_by(const double dx, const double dy, const double d) {
  const double scaleFactor = d / screenDist;
  const Vector3D& displacement =
    c2w[0] * (dx * scaleFactor) + c2w[1] * (dy * scaleFactor);
  pos += displacement;
  targetPos += displacement;
}

void Camera::move_forward(const double dist) {
  double newR = min(max(r - dist, minR), maxR);
  pos = targetPos + ((pos - targetPos) * (newR / r));
  r = newR;
}

void Camera::rotate_by(const double dPhi, const double dTheta) {
  phi = clamp(phi + dPhi, 0.0, (double) PI);
  theta += dTheta;
  compute_position();
}

void Camera::compute_position() {
  double sinPhi = sin(phi);
  if (sinPhi == 0) {
    phi += EPS_F;
    sinPhi = sin(phi);
  }
  const Vector3D dirToCamera(r * sinPhi * sin(theta),
                             r * cos(phi),
                             r * sinPhi * cos(theta));
  pos = targetPos + dirToCamera;
  Vector3D upVec(0, sinPhi > 0 ? 1 : -1, 0);
  Vector3D screenXDir = cross(upVec, dirToCamera);
  screenXDir.normalize();
  Vector3D screenYDir = cross(dirToCamera, screenXDir);
  screenYDir.normalize();

  c2w[0] = screenXDir;
  c2w[1] = screenYDir;
  c2w[2] = dirToCamera.unit();   // camera's view direction is the
                                 // opposite of of dirToCamera, so
                                 // directly using dirToCamera as
                                 // column 2 of the matrix takes [0 0 -1]
                                 // to the world space view direction
}

void Camera::dump_settings(string filename) {
  ofstream file(filename);
  file << hFov << " " << vFov << " " << ar << " " << nClip << " " << fClip << endl;
  for (int i = 0; i < 3; ++i)
    file << pos[i] << " ";
  for (int i = 0; i < 3; ++i)
    file << targetPos[i] << " ";
  file << endl;
  file << phi << " " << theta << " " << r << " " << minR << " " << maxR << endl;
  for (int i = 0; i < 9; ++i)
    file << c2w(i/3, i%3) << " ";
  file << endl;
  file << screenW << " " << screenH << " " << screenDist << endl;
  file << focalDistance << " " << lensRadius << endl;
  cout << "[Camera] Dumped settings to " << filename << endl;
}

void Camera::load_settings(string filename) {
  ifstream file(filename);

  file >> hFov >> vFov >> ar >> nClip >> fClip;
  for (int i = 0; i < 3; ++i)
    file >> pos[i];
  for (int i = 0; i < 3; ++i)
    file >> targetPos[i];
  file >> phi >> theta >> r >> minR >> maxR;
  for (int i = 0; i < 9; ++i)
    file >> c2w(i/3, i%3);
  file >> screenW >> screenH >> screenDist;
  file >> focalDistance >> lensRadius;
  cout << "[Camera] Loaded settings from " << filename << endl;
}

Ray Camera::generate_ray(double x, double y) const {

  // Part 1, Task 2:
  // compute position of the input sensor sample coordinate on the
  // canonical sensor plane one unit away from the pinhole.
  // Note: hFov and vFov are in degrees.
  // 

  Vector3D bottomLeft = Vector3D(-tan(radians(hFov)*.5), -tan(radians(vFov)*.5),-1);
  Vector3D topRight = Vector3D( tan(radians(hFov)*.5),  tan(radians(vFov)*.5),-1);

  double newX = bottomLeft.x * (1.0 - x) + topRight.x * x;
  double newY = bottomLeft.y * (1.0 - y) + topRight.y * y;

  Vector3D r_d = Vector3D(newX, newY, -1);

  r_d = c2w * r_d;
  r_d.normalize();

  Ray r = Ray(pos, r_d);
  r.min_t = nClip;
  r.max_t = fClip;

  return r;

}

Ray Camera::generate_ray_for_thin_lens(double x, double y, double rndR, double rndTheta) const {
  //Todo 3-2, Task 4:
  // compute position and direction of ray from the input sensor sample coordinate.
  // Note: use rndR and rndTheta to uniformly sample a unit disk.
  
  Vector3D bottomLeft = Vector3D(-tan(radians(hFov)*.5), -tan(radians(vFov)*.5),-1);
  Vector3D topRight = Vector3D( tan(radians(hFov)*.5),  tan(radians(vFov)*.5),-1);

  double newX = bottomLeft.x * (1.0 - x) + topRight.x * x;
  double newY = bottomLeft.y * (1.0 - y) + topRight.y * y;

  Vector3D r_d = Vector3D(newX, newY, -1);

  Vector3D pLens = Vector3D(lensRadius * sqrt(rndR) * cos(2.0 * PI * rndTheta), lensRadius * sqrt(rndR) * sin(2.0 * PI * rndTheta), 0.0);
  Vector3D pFocus = r_d * (focalDistance);

  pLens = c2w * pLens;
  Vector3D dir = (pFocus - pLens).unit();
  dir = c2w * dir;

  Ray r = Ray(pLens + pos, dir);
  r.min_t = nClip;
  r.max_t = fClip;

  return r;
}


Ray Camera::generate_ray_for_microlens(double x, double y, double rndR, double rndTheta, double rngMicroR, double rngMicroTheta) const {
  Vector3D bottomLeft = Vector3D(-tan(radians(hFov)*.5), -tan(radians(vFov)*.5),-1);
  Vector3D topRight = Vector3D( tan(radians(hFov)*.5),  tan(radians(vFov)*.5),-1);

  double newX = bottomLeft.x * (1.0 - x) + topRight.x * x;
  double newY = bottomLeft.y * (1.0 - y) + topRight.y * y;

  double microlensRadius = lensRadius / 1000.0;

  // new start location to the red vector
  Vector3D pMicroLens = Vector3D(microlensRadius * sqrt(rndMicroR) * cos(2.0 * PI * rndMicroR), microlensRadius * sqrt(rndMicroR) * sin(2.0 * PI * rndMicroR), 0.0);
  
  double mlX = pMicroLens.x;
  double mlY = pMicroLens.y;
  // Direction from microlens to center of main lens
  Vector3D r_d = Vector3D(mlX, mlY, -focalDistance);

  // location the ray hits on the main lens
  Vector3D pLens = Vector3D(lensRadius * sqrt(rndR) * cos(2.0 * PI * rndTheta), lensRadius * sqrt(rndR) * sin(2.0 * PI * rndTheta), -focalDistance);
  // location where the ray hits the object
  Vector3D pFocus = pMicroLens + r_d * (focalDistance);

  pLens = c2w * pLens;
  Vector3D dir = (pFocus - pLens).unit();
  dir = c2w * dir;

  Ray r = Ray(pLens + pos, dir);
  r.min_t = nClip;
  r.max_t = fClip;

  return r;
}


} // namespace CGL
