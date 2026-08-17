#include "bitbot_mujoco/device/mujoco_depth_camera.hpp"

#include "bitbot_kernel/kernel/config_parser.h"
#include "bitbot_kernel/device/device_factory.hpp"

#include <GLFW/glfw3.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cstring>

namespace bitbot
{
  // Self-registration: makes <device type="MujocoDepthCamera" .../> usable in
  // the kernel config without modifying bitbot_mujoco itself.
  static DeviceRegistrar<MujocoDevice, MujocoDepthCamera> mujoco_depth_camera_registrar(
      MUJOCO_DEPTH_CAMERA_TYPE, "MujocoDepthCamera");

  MujocoDepthCamera::MujocoDepthCamera(const pugi::xml_node& device_node)
    : MujocoDevice(device_node)
  {
    basic_type_ = (uint32_t)BasicDeviceType::USER_DEFINE;
    type_ = (uint32_t)MUJOCO_DEPTH_CAMERA_TYPE;

    monitor_header_.headers = {"width", "height", "frequency"};
    monitor_data_.resize(monitor_header_.headers.size());

    ConfigParser::ParseAttribute2s(camera_name_, device_node.attribute("camera"));
    if(camera_name_.empty())
      camera_name_ = name_;

    ConfigParser::ParseAttribute2i(width_, device_node.attribute("width"));
    ConfigParser::ParseAttribute2i(height_, device_node.attribute("height"));
    ConfigParser::ParseAttribute2d(frequency_, device_node.attribute("frequency"));
    ConfigParser::ParseAttribute2d(min_depth_, device_node.attribute("min_depth"));
    ConfigParser::ParseAttribute2d(max_depth_, device_node.attribute("max_depth"));
    ConfigParser::ParseAttribute2d(znear_, device_node.attribute("znear"));
    ConfigParser::ParseAttribute2d(zfar_, device_node.attribute("zfar"));
    ConfigParser::ParseAttribute2d(noise_stddev_, device_node.attribute("noise_stddev"));
    ConfigParser::ParseAttribute2i(crop_top_, device_node.attribute("crop_top"));
    ConfigParser::ParseAttribute2i(crop_bottom_, device_node.attribute("crop_bottom"));
    ConfigParser::ParseAttribute2i(crop_left_, device_node.attribute("crop_left"));
    ConfigParser::ParseAttribute2i(crop_right_, device_node.attribute("crop_right"));
    ConfigParser::ParseAttribute2i(blur_kernel_, device_node.attribute("blur_kernel"));
    ConfigParser::ParseAttribute2d(blur_sigma_, device_node.attribute("blur_sigma"));
    ConfigParser::ParseAttribute2b(normalize_, device_node.attribute("normalize"));
    ConfigParser::ParseAttribute2b(debug_vis_, device_node.attribute("debug_vis"));
    ConfigParser::ParseAttribute2b(debug_vis_raw_, device_node.attribute("debug_vis_raw"));

    obs_width_ = width_ - crop_left_ - crop_right_;
    obs_height_ = height_ - crop_top_ - crop_bottom_;
    if(width_ <= 0 || height_ <= 0 || obs_width_ <= 0 || obs_height_ <= 0)
    {
      logger_->error("MujocoDepthCamera id:{} has invalid resolution {}x{}.", id_, width_, height_);
      return;
    }
    if(max_depth_ <= min_depth_)
    {
      logger_->error("MujocoDepthCamera id:{} needs min_depth < max_depth, got {} {}.", id_, min_depth_, max_depth_);
      return;
    }
    if(frequency_ <= 0)
      frequency_ = 10.0;

    // Devices are constructed on the main thread (inside the kernel
    // constructor), which is the only thread GLFW allows window creation on.
    // The hidden window merely provides the OpenGL context; frames are
    // rendered on the physics thread in Input().
    if(!glfwInit())
    {
      logger_->error("MujocoDepthCamera id:{}: glfwInit failed.", id_);
      return;
    }
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_FALSE);
    gl_window_ = glfwCreateWindow(640, 480, "mujoco_depth_camera", nullptr, nullptr);
    // GLFW window hints are sticky: restore the defaults so the simulate GUI
    // window created later does not inherit the hidden single-buffered setup
    // (it only resets GLFW_SAMPLES and GLFW_VISIBLE itself).
    glfwDefaultWindowHints();
    if(!gl_window_)
    {
      logger_->error("MujocoDepthCamera id:{}: could not create the hidden GLFW window.", id_);
      return;
    }
    // Release the context so the physics thread can make it current.
    glfwMakeContextCurrent(nullptr);

    logger_->info("MujocoDepthCamera {} camera:{} resolution:{}x{} (obs {}x{}) frequency:{} Hz depth range:[{}, {}] m",
                  name_, camera_name_, width_, height_, obs_width_, obs_height_, frequency_, min_depth_, max_depth_);
  }

  MujocoDepthCamera::~MujocoDepthCamera()
  {
    // No GL teardown: devices are destroyed after the GUI has terminated
    // GLFW (render loop returns before the kernel is destructed), so the
    // context is already gone.
  }

  void MujocoDepthCamera::UpdateModel(const mjModel* m, mjData* mj_d)
  {
    if(!gl_window_)
      return;

    camera_id_ = mj_name2id(m, mjOBJ_CAMERA, camera_name_.c_str());
    if(camera_id_ == -1)
    {
      logger_->error("MujocoDepthCamera {}: can not find camera named {} in the mujoco model.", name_, camera_name_);
      return;
    }

    // The offscreen buffer size is a model parameter; grow it to the
    // configured resolution. The kernel only hands out a const model.
    mjModel* model = const_cast<mjModel*>(m);
    if(model->vis.global.offwidth < width_)
      model->vis.global.offwidth = width_;
    if(model->vis.global.offheight < height_)
      model->vis.global.offheight = height_;

    // Optional near/far plane override, vis.map values are relative to the
    // model extent. Note: this also affects the interactive view.
    if(znear_ > 0)
      model->vis.map.znear = znear_ / m->stat.extent;
    if(zfar_ > 0)
      model->vis.map.zfar = zfar_ / m->stat.extent;
    znear_abs_ = m->vis.map.znear * m->stat.extent;
    zfar_abs_ = m->vis.map.zfar * m->stat.extent;

    glfwMakeContextCurrent(gl_window_);

    mjv_defaultCamera(&cam_);
    mjv_defaultOption(&opt_);
    mjv_defaultScene(&scn_);
    mjv_makeScene(m, &scn_, 2000);
    mjr_defaultContext(&con_);
    mjr_makeContext(m, &con_, 200);
    mjr_setBuffer(mjFB_OFFSCREEN, &con_);
    if(con_.currentBuffer != mjFB_OFFSCREEN)
    {
      logger_->error("MujocoDepthCamera {}: offscreen rendering not supported.", name_);
      glfwMakeContextCurrent(nullptr);
      return;
    }

    cam_.type = mjCAMERA_FIXED;
    cam_.fixedcamid = camera_id_;

    mjrRect max_viewport = mjr_maxViewport(&con_);
    viewport_ = {0, 0, width_, height_};
    if(viewport_.width > max_viewport.width || viewport_.height > max_viewport.height)
    {
      logger_->warn("MujocoDepthCamera {}: offscreen buffer {}x{} smaller than requested {}x{}, clamping.",
                    name_, max_viewport.width, max_viewport.height, width_, height_);
      viewport_.width = std::min(viewport_.width, max_viewport.width);
      viewport_.height = std::min(viewport_.height, max_viewport.height);
    }

    depth_buffer_.resize(width_ * height_);
    depth_obs_.resize(obs_width_ * obs_height_);
    ready_ = true;

    glfwMakeContextCurrent(nullptr);

    logger_->debug("MujocoDepthCamera {} camera_id:{} znear:{} zfar:{}", name_, camera_id_, znear_abs_, zfar_abs_);
  }

  void MujocoDepthCamera::Input(const mjModel* m, mjData* mj_d)
  {
    if(!ready_)
      return;

    // Frame gating on simulation time; a negative difference means the
    // simulation was reset, so render again.
    double dt_sim = mj_d->time - last_render_time_;
    if(dt_sim < 1.0 / frequency_ && dt_sim >= 0)
      return;
    last_render_time_ = mj_d->time;

    auto start = std::chrono::steady_clock::now();
    Render(m, mj_d);

    if(frame_count_ > 0)
    {
      double dt_wall = std::chrono::duration<double>(start - last_frame_wall_time_).count();
      if(dt_wall > 0)
        measured_frequency_ =
            measured_frequency_ * (frame_count_ - 1) / frame_count_ + (1.0 / dt_wall) / frame_count_;
    }
    last_frame_wall_time_ = start;
    frame_count_++;
  }

  void MujocoDepthCamera::Render(const mjModel* m, mjData* mj_d)
  {
    glfwMakeContextCurrent(gl_window_);

    // mjCAT_STATIC | mjCAT_DYNAMIC: real geometry only, no decorations
    mjv_updateScene(m, mj_d, &opt_, nullptr, &cam_, mjCAT_STATIC | mjCAT_DYNAMIC, &scn_);
    mjr_render(viewport_, &scn_, &con_);
    mjr_readPixels(nullptr, depth_buffer_.data(), viewport_, &con_);

    glfwMakeContextCurrent(nullptr);

    // OpenGL depth z in [0, 1] (znear .. zfar, mjrContext defaults to
    // mjDEPTH_ZERONEAR) -> metric depth; z == 1 (sky) -> zfar.
    cv::Mat depth_image(height_, width_, CV_32FC1, depth_buffer_.data());
    std::normal_distribution<float> noise(0.0f, (float)noise_stddev_);
    for(int i = 0; i < depth_image.rows; ++i)
    {
      float* row = depth_image.ptr<float>(i);
      for(int j = 0; j < depth_image.cols; ++j)
      {
        float z = row[j];
        float metric = z >= 1.0f
                           ? zfar_abs_
                           : znear_abs_ / (1.0f - z * (1.0f - znear_abs_ / zfar_abs_));
        if(noise_stddev_ > 0)
          metric += noise(noise_rng_);
        metric = std::min(std::max(metric, (float)min_depth_), (float)max_depth_);
        row[j] = normalize_ ? (metric - min_depth_) / (max_depth_ - min_depth_) : metric;
      }
    }
    // OpenGL rows are bottom-up
    cv::flip(depth_image, depth_image, 0);

    cv::Mat processed = depth_image;
    if(crop_top_ || crop_bottom_ || crop_left_ || crop_right_)
      processed = depth_image(cv::Rect(crop_left_, crop_top_, obs_width_, obs_height_));
    if(blur_kernel_ > 0)
    {
      cv::Mat blurred;
      cv::GaussianBlur(processed, blurred, cv::Size(blur_kernel_, blur_kernel_), blur_sigma_,
                       blur_sigma_, cv::BORDER_REFLECT);
      processed = blurred;
    }
    // Cropping without blur leaves a non-continuous view of depth_image
    if(!processed.isContinuous())
    {
      cv::Mat continuous;
      processed.copyTo(continuous);
      processed = continuous;
    }
    std::memcpy(depth_obs_.data(), processed.ptr<float>(0), depth_obs_.size() * sizeof(float));

    if(frame_count_ == 0)
    {
      auto minmax = std::minmax_element(depth_obs_.begin(), depth_obs_.end());
      double scale = normalize_ ? (max_depth_ - min_depth_) : 0.0;
      double offset = normalize_ ? min_depth_ : 0.0;
      logger_->info("MujocoDepthCamera {} first depth frame: obs range [{:.3f}, {:.3f}] m",
                    name_, *minmax.first * scale + offset, *minmax.second * scale + offset);
    }

    if(debug_vis_)
    {
      cv::Mat img8u;
      processed.convertTo(img8u, CV_8UC1, 255.0);
      cv::Mat img_big;
      cv::resize(img8u, img_big, cv::Size(img8u.cols * 5, img8u.rows * 5), 0, 0, cv::INTER_NEAREST);
      cv::imshow("depth_camera: " + name_, img_big);

      // if(frame_count_ % 60 == 0)
      //   cv::imwrite("/tmp/depth_debug_obs.png", img_big);
    }

    // Full frame before cropping: the observation crops the upper part of
    // the image (gazebo pipeline), which hides distant obstacles.
    if(debug_vis_raw_)
    {
      cv::Mat full8u, full_big;
      depth_image.convertTo(full8u, CV_8UC1, 255.0);
      cv::resize(full8u, full_big, cv::Size(full8u.cols * 8, full8u.rows * 8), 0, 0, cv::INTER_NEAREST);
      cv::imshow("depth_camera full: " + name_, full_big);

      // if(frame_count_ % 60 == 0)
      //   cv::imwrite("/tmp/depth_debug_full.png", full_big);
    }

    if(debug_vis_ || debug_vis_raw_)
      cv::waitKey(1);
  }

  void MujocoDepthCamera::Output(const mjModel* m, mjData* mj_d)
  {
    return;
  }

  void MujocoDepthCamera::UpdateRuntimeData()
  {
    monitor_data_[0] = obs_width_;
    monitor_data_[1] = obs_height_;
    monitor_data_[2] = measured_frequency_;
  }

  cv::Mat MujocoDepthCamera::GetDepthImage() const
  {
    cv::Mat image(obs_height_, obs_width_, CV_32FC1);
    std::memcpy(image.data, depth_obs_.data(), depth_obs_.size() * sizeof(float));
    return image;
  }
}
