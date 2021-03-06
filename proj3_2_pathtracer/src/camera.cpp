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


Ray Camera::generate_ray_for_microlens(double x, double y, double originX, double originY, double rndR, double rndTheta) const {
  Vector3D bottomLeft = Vector3D(-tan(radians(hFov)*.5), -tan(radians(vFov)*.5),-1);
  Vector3D topRight = Vector3D( tan(radians(hFov)*.5),  tan(radians(vFov)*.5),-1);

  double newX = bottomLeft.x * (1.0 - x) + topRight.x * x;
  double newY = bottomLeft.y * (1.0 - y) + topRight.y * y;

  // For light field storage
  double s = originX;
  double t = originY;

  Vector3D r_d = Vector3D(newX, newY, -1);

  Vector3D pLens = Vector3D(lensRadius * sqrt(rndR) * cos(2.0 * PI * rndTheta), lensRadius * sqrt(rndR) * sin(2.0 * PI * rndTheta), 0.0);
  Vector3D pFocus = r_d * (focalDistance);

  // bucket of the lens (or the microlens location)
  double u = round((num_microlenses_wide * (pLens.x + lensRadius) - fmod(num_microlenses_wide * (pLens.x + lensRadius), lensRadius * 2.0)) / (lensRadius * 2.0));
  double v = round((num_microlenses_wide * (pLens.y + lensRadius) - fmod(num_microlenses_wide * (pLens.y + lensRadius), lensRadius * 2.0)) / (lensRadius * 2.0));

  pLens = c2w * pLens;
  Vector3D dir = (pFocus - pLens).unit();
  dir = c2w * dir;

  Ray r = Ray(pLens + pos, dir);
  r.min_t = nClip;
  r.max_t = fClip;

  r.u = u;
  r.v = v;
  r.s = s;
  r.t = t;

  return r;
}

//
// WE WILL NEED TO MAKE SURE THAT THE NUM_MICROLENSES_WIDE CAN BE SET FROM THE COMMAND LINE
//

void Camera::update_lightField(double u, double v, double s, double t, Spectrum spec) {
  // See if lightField(u, v, s, t) exists
  if (lightField.find(u) != lightField.end()) {
    if (lightField[u].find(v) != lightField[u].end()) {
      if (lightField[u][v].find(s) != lightField[u][v].end()) {
        if (lightField[u][v][s].find(t) != lightField[u][v][s].end()) {
          // it exists
          lightField[u][v][s][t] = std::pair<int, Spectrum> (std::get<0>(lightField[u][v][s][t]) + 1, std::get<1>(lightField[u][v][s][t]) + spec);
        } else {
          lightField[u][v][s][t] = std::pair<int, Spectrum> (1, spec);
        }
      } 
      else {
        lightField[u][v][s][t] = std::pair<int, Spectrum> (1, spec);
      }
    } else {
      lightField[u][v][s][t] = std::pair<int, Spectrum> (1, spec);
    }
  } else {
    lightField[u][v][s][t] = std::pair<int, Spectrum> (1, spec);
  }
}

void Camera::refocused_lightField(double newFocalDistance, double bufferWidth, double bufferHeight) {
  std::map<double, std::map<double, std::map<double, std::map<double, std::pair<int, Spectrum>>>>> newLightField;
  Vector3D destination;
  // i is u, j is v, y is t, x is s
  for (double i = 0.0; i < num_microlenses_wide; i++) {
    for (double j = 0.0; j < num_microlenses_wide; j++) {
      for (double y = 0.0; y < bufferHeight; y++) {
        for (double x = 0.0; x < bufferWidth; x++) {
          // if ((i == 8 || i == 9 || i == 11 || i == 13 || i == 15) && (lightField.find(i) != lightField.end() && lightField[i].find(j) != lightField[i].end() && lightField[i][j].find(x) != lightField[i][j].end() && lightField[i][j][x].find(y) != lightField[i][j][x].end())) {
          //   printf("i, j, x, y are %f, %f, %f, %f \n", i, j, x, y);
          // }
          if (lightField.find(i) == lightField.end() || lightField[i].find(j) == lightField[i].end() || lightField[i][j].find(x) == lightField[i][j].end() || lightField[i][j][x].find(y) == lightField[i][j][x].end()) {
            continue;
          }
          // need to be the camera coordinates of the actual buckets, not the bucket index

          // convert to world coordinates
          double lensX = (i * 2 * lensRadius) / (num_microlenses_wide) - lensRadius;
          double lensY = (j * 2 * lensRadius) / (num_microlenses_wide) - lensRadius;

          Vector3D pLens = Vector3D(lensX, lensY, 0.0);

          Vector3D bottomLeft = Vector3D(-tan(radians(hFov)*.5), -tan(radians(vFov)*.5), -1);
          Vector3D topRight = Vector3D( tan(radians(hFov)*.5),  tan(radians(vFov)*.5), -1);

          double newX = bottomLeft.x * (1.0 - (x / bufferWidth)) + topRight.x * (x / bufferWidth);
          double newY = bottomLeft.y * (1.0 - (y / bufferHeight)) + topRight.y * (y / bufferHeight);

          Vector3D pSensor = Vector3D(newX, newY, 1.0);

          Vector3D r_o = pLens;
          Vector3D r_d = pSensor - pLens;

          Ray r = Ray(r_o, r_d);
          
          destination = r.o + newFocalDistance * r.d;
          // need to convert destination coordinates back

          //Start of trial
          Vector3D bottomLeftOfLens = Vector3D(-lensRadius, -lensRadius, 0);
          Vector3D topRightOfLens = Vector3D(lensRadius, lensRadius, 0);
          Vector3D newBottomLeft = (topRightOfLens + newFocalDistance * (Vector3D(bottomLeft.x, bottomLeft.y, 1) - topRightOfLens));
          Vector3D newTopRight = (bottomLeftOfLens + newFocalDistance * (Vector3D(topRight.x, topRight.y, 1) - bottomLeftOfLens));

          double newBufferWidth = (newTopRight.x - newBottomLeft.x) / (topRight.x - bottomLeft.x) * bufferWidth;
          double newBufferHeight = (newTopRight.y - newBottomLeft.y) / (topRight.y - bottomLeft.y) * bufferHeight;

          double revertedX = ((destination.x - newBottomLeft.x) * newBufferWidth) / (newTopRight.x - newBottomLeft.x);
          double revertedY = ((destination.y - newBottomLeft.y) * newBufferHeight) / (newTopRight.y - newBottomLeft.y);

          revertedX = trunc(revertedX * (bufferWidth / newBufferWidth));
          revertedY = trunc(revertedY * (bufferHeight / newBufferHeight));
          // End of trial

          // double revertedX = trunc(((destination.x - bottomLeft.x) * bufferWidth) / (topRight.x - bottomLeft.x));
          // double revertedY = trunc(((destination.y - bottomLeft.y) * bufferHeight) / (topRight.y - bottomLeft.y));

          // if (revertedX >= bufferWidth || revertedX < 0 || revertedY >= bufferHeight || revertedY < 0) {
          //   continue;
          // }
          // if (i == 9.0 || i == 11.0 || i == 13.0 || i == 15.0) {
          //   printf("After revert setting [%f][%f][%f][%f] equal to [%f][%f][%f][%f]\n", i, j, revertedX, revertedY, i, j, x, y);
          // }
          newLightField[i][j][revertedX][revertedY] = lightField[i][j][x][y];
        }
      }
    }
  }
  lightField = std::map<double, std::map<double, std::map<double, std::map<double, std::pair<int, Spectrum>>>>>();
  lightField = newLightField; 
  printf("recreated light field!\n");
}

} // namespace CGL





















