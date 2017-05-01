#ifndef CGL_CAMERA_H
#define CGL_CAMERA_H

#include <iostream>
#include <map>

#include "collada/camera_info.h"
#include "CGL/matrix3x3.h"

#include "math.h"
#include "ray.h"


namespace CGL {

/**
 * Camera.
 */
class Camera {
 public:
  /*
    Sets the field of view to match screen screenW/H.
    NOTE: data and screenW/H will almost certainly disagree about the aspect
          ratio. screenW/H are treated as the source of truth, and the field
          of view is expanded along whichever dimension is too narrow.
    NOTE2: info.hFov and info.vFov are expected to be in DEGREES.
  */
  void configure(const Collada::CameraInfo& info,
                 size_t screenW, size_t screenH);

  /*
    Phi and theta are in RADIANS.
  */
  void place(const Vector3D& targetPos, const double phi, const double theta,
             const double r, const double minR, const double maxR);

  string param_string() {
    return "";
  }

  /*
    Copies just placement data from the other camera.
  */
  void copy_placement(const Camera& other);

  /*
    Updates the screen size to be the specified size, keeping screenDist
    constant.
  */
  void set_screen_size(const size_t screenW, const size_t screenH);

  /*
    Translates the camera such that a value at distance d directly in front of
    the camera moves by (dx, dy). Note that dx and dy are in screen coordinates,
    while d is in world-space coordinates (like pos/dir/up).
  */
  void move_by(const double dx, const double dy, const double d);

  /*
    Move the specified amount along the view axis.
  */
  void move_forward(const double dist);

  /*
    Rotate by the specified amount around the target.
  */
  void rotate_by(const double dPhi, const double dTheta);

  Vector3D position() const { return pos; }
  Vector3D view_point() const { return targetPos; }
  Vector3D up_dir() const { return c2w[1]; }
  double v_fov() const { return vFov; }
  double aspect_ratio() const { return ar; }
  double near_clip() const { return nClip; }
  double far_clip() const { return fClip; }

  virtual void dump_settings(std::string filename);
  virtual void load_settings(std::string filename);

  /**
   * Returns a world-space ray from the camera that corresponds to a
   * ray exiting the camera that deposits light at the sensor plane
   * position given by (x,y).  x and y are provided in the normalized
   * coordinate space of the sensor.  For example (0.5, 0.5)
   * corresponds to the middle of the screen.
   *
   * \param x x-coordinate of the ray sample in the view plane
   * \param y y-coordinate of the ray sample in the view plane
   */
  Ray generate_ray(double x, double y) const;
  Ray generate_ray_for_thin_lens(double x, double y, double rndR, double rndTheta) const;
  Ray generate_ray_for_microlens(double x, double y, double originX, double originY, double rndR, double rndTheta)  const;

  // Lens aperture and focal distance for depth of field effects.
  double lensRadius;
  double focalDistance;
  double num_microlenses_wide;

  // light field
  std::map<double, std::map<double, std::map<double, std::map<double, std::pair<int, Spectrum>>>>> lightField;

  void update_lightField(double u, double v, double s, double t, Spectrum spec);
  std::map<double, std::map<double, std::map<double, std::map<double, std::pair<int, Spectrum>>>>> refocused_lightField(double newFocalDistance);

 private:
  // Computes pos, screenXDir, screenYDir from target, r, phi, theta.
  void compute_position();

  // Field of view aspect ratio, clipping planes.
  double hFov, vFov, ar, nClip, fClip;

  // Current position and target point (the point the camera is looking at).
  Vector3D pos, targetPos;

  // Orientation relative to target, and min & max distance from the target.
  double phi, theta, r, minR, maxR;

  // camera-to-world rotation matrix (note: also need to translate a
  // camera-space point by 'pos' to perform a full camera-to-world
  // transform)
  Matrix3x3 c2w;

  // Info about screen to render to; it corresponds to the camera's full field
  // of view at some distance.
  size_t screenW, screenH;
  double screenDist;
};

} // namespace CGL

#endif // CGL_CAMERA_H
