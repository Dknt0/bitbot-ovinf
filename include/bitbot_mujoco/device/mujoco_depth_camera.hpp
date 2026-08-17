#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <opencv2/core.hpp>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "bitbot_mujoco/device/mujoco_device.hpp"

struct GLFWwindow;

namespace bitbot {
// Follows bitbot::MujocoDeviceType (11000 - 11005 are used by bitbot_mujoco).
constexpr uint32_t MUJOCO_DEPTH_CAMERA_TYPE = 11006;

/**
 * @brief Simulated depth camera, renders the scene of an MJCF <camera> into
 *        an offscreen buffer of a private OpenGL context.
 *
 * The hidden GLFW window is created in the constructor (main thread, as
 * required by GLFW). Rendering happens on the physics thread inside
 * Input(), gated on simulation time. Everything after mjr_readPixels
 * (depth conversion, noise, crop, blur, debug windows) runs on a separate
 * processing thread which always works on the newest frame and skips
 * stale ones; consumers read the newest processed observation. The output
 * pipeline mirrors the training / real robot depth processing.
 *
 * XML attributes:
 *   camera       name of the MJCF <camera> (default: device name)
 *   width        image width  [px]
 *   height       image height [px]
 *   frequency    frame rate [Hz], gated on simulation time
 *   min_depth    near clip of the output [m]
 *   max_depth    far clip of the output [m]; sky / far pixels map to it
 *   znear        OpenGL near plane [m], 0: model default (vis.map.znear *
 * extent) zfar         OpenGL far  plane [m], 0: model default; a small value
 *                improves depth precision but also clips the interactive view
 *   noise_stddev gaussian noise on the metric depth [m], 0 disables
 *   crop_top     pixels cropped before blur [px]
 *   crop_bottom  pixels cropped before blur [px]
 *   crop_left    pixels cropped before blur [px]
 *   crop_right   pixels cropped before blur [px]
 *   blur_kernel  Gaussian blur kernel size, 0 disables blur [px]
 *   blur_sigma   Gaussian blur sigma
 *   normalize    map [min_depth, max_depth] -> [0, 1]; false: metric meters
 *   debug_vis    show the processed depth observation in an OpenCV window
 *   debug_vis_raw show the full (uncropped) depth image in an OpenCV window
 */
class MujocoDepthCamera final : public MujocoDevice {
 public:
  MujocoDepthCamera(const pugi::xml_node& device_node);
  ~MujocoDepthCamera();

  /// Newest processed depth observation, row-major,
  /// GetObsWidth()*GetObsHeight()
  std::vector<float> GetDepthObs() const;

  /// Copy of the newest processed depth image, CV_32FC1
  cv::Mat GetDepthImage() const;

  int GetObsWidth() const { return obs_width_; }

  int GetObsHeight() const { return obs_height_; }

  /// Measured frame rate [Hz], running average
  double GetFrequency() const { return measured_frequency_; }

 private:
  virtual void UpdateModel(const mjModel* m, mjData* mj_d) final;
  virtual void Input(const mjModel* m, mjData* mj_d) final;
  virtual void Output(const mjModel* m, mjData* mj_d) final;
  virtual void UpdateRuntimeData() final;

  // Physics thread: GL render + pixel read, hands the raw frame over.
  void Render(const mjModel* m, mjData* mj_d);
  // Processing thread: waits for frames, keeps the newest one.
  void ProcessingThread();
  // Conversion -> noise -> clip -> normalize -> flip -> crop -> blur -> obs
  void ProcessFrame(std::vector<float>& raw);
  void StopProcessingThread();

  // configuration
  std::string camera_name_;
  int width_ = 106;
  int height_ = 60;
  double frequency_ = 10.0;
  double min_depth_ = 0.0;
  double max_depth_ = 2.5;
  double znear_ = 0.0;
  double zfar_ = 0.0;
  double noise_stddev_ = 0.0;
  int crop_top_ = 0;
  int crop_bottom_ = 0;
  int crop_left_ = 0;
  int crop_right_ = 0;
  int blur_kernel_ = 3;
  double blur_sigma_ = 1.0;
  bool normalize_ = true;
  bool debug_vis_ = false;
  bool debug_vis_raw_ = false;

  // opengl
  GLFWwindow* gl_window_ = nullptr;

  // mujoco rendering
  mjvScene scn_;
  mjvCamera cam_;
  mjvOption opt_;
  mjrContext con_;
  mjrRect viewport_ = {0, 0, 0, 0};
  int camera_id_ = -1;
  bool ready_ = false;
  float znear_abs_ = 0.01f;
  float zfar_abs_ = 50.0f;

  // data
  std::vector<float> depth_buffer_;  // raw depth of one rendered frame
  std::vector<float> depth_obs_;     // processed observation (obs_mutex_)
  std::mt19937 noise_rng_{42};
  int obs_width_ = 0;
  int obs_height_ = 0;
  double last_render_time_ = -1.0;

  uint64_t frame_count_ = 0;
  double measured_frequency_ = 0.0;
  std::chrono::steady_clock::time_point last_frame_wall_time_;

  // processing thread
  std::thread processing_thread_;
  std::atomic<bool> processing_running_{false};
  std::mutex frame_mutex_;  // guards raw_latest_ / new_frame_
  std::condition_variable frame_cv_;
  bool new_frame_ = false;
  std::vector<float> raw_latest_;  // newest raw frame (frame_mutex_)
  std::vector<float> raw_work_;    // processing-thread-private
  mutable std::mutex obs_mutex_;   // guards depth_obs_ / processed_count_
  uint64_t processed_count_ = 0;
};
}  // namespace bitbot
