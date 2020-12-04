/// Copyright (c) 2014,
/// Systems, Robotics and Vision Group
/// University of the Balearic Islands
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are met:
///     * Redistributions of source code must retain the above copyright
///       notice, this list of conditions and the following disclaimer.
///     * Redistributions in binary form must reproduce the above copyright
///       notice, this list of conditions and the following disclaimer in the
///       documentation and/or other materials provided with the distribution.
///     * All advertising materials mentioning features or use of this software
///       must display the following acknowledgement:
///       This product includes software developed by
///       Systems, Robotics and Vision Group, Univ. of the Balearic Islands
///     * Neither the name of Systems, Robotics and Vision Group, University of
///       the Balearic Islands nor the names of its contributors may be used
///       to endorse or promote products derived from this software without
///       specific prior written permission.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
/// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
/// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
/// ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
/// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
/// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
/// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
/// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
/// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
/// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <avt_vimba_camera/mono_camera.h>

#define DEBUG_PRINTS 1

namespace avt_vimba_camera {

MonoCamera::MonoCamera(const rclcpp::NodeOptions &node_options) : 
    Node("avt_vimba_camera", node_options)
    //nh_(nh), nhp_(nhp), it_(nhp), cam_(ros::this_node::getName()) 
{
    // Start Vimba & list all available cameras
    api_.start();

    // set up config from parms
    //node_handle_ = rclcpp::Node::make_shared("avt_vimba_camera");
    //camera_parms_ = std::make_shared<avt_vimba_camera::AvtVimbaCameraParms>(node_handle_);
    //std::cout << "Test: " << camera_parms_->frame_id_descriptor.description << std::endl;
    
    // create an image publisher w/QoS profile
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_sensor_data;
    custom_qos_profile.depth = 20;  // TEST
    camera_info_pub_ = image_transport::create_camera_publisher(this, "image", custom_qos_profile);

    RCLCPP_INFO(get_logger(), "Test message");

    info_man_ = std::make_shared<camera_info_manager::CameraInfoManager>(this);

#if 0 // [neil-rti] this was a test; calibration is loaded in configure() call tree
    /* get ROS2 config parameter for camera calibration file */
    auto camera_calibration_file_param_ = this->declare_parameter("camera_calibration_file", "file://config/camera.yaml");
    cinfo_manager_->loadCameraInfo(camera_calibration_file_param_);
#endif  //np

    last_frame_ = std::chrono::steady_clock::now();

  // Set the frame callback
  cam_.setCallback(std::bind(&avt_vimba_camera::MonoCamera::frameCallback, this, std::placeholders::_1));


#if 0 //np
  // Set the params
  nhp_.param("ip", ip_, std::string(""));
  nhp_.param("guid", guid_, std::string(""));
  nhp_.param("camera_info_url", camera_info_url_, std::string(""));
  std::string frame_id;
  nhp_.param("frame_id", frame_id, std::string(""));
  nhp_.param("show_debug_prints", show_debug_prints_, false);
#endif  //np

  // [neil-rti] test w/default config
    camera_config_.frame_id = std::string("camera");
    camera_config_.trig_timestamp_topic = std::string("");
    camera_config_.acquisition_mode = std::string("Continuous");
    camera_config_.acquisition_rate = (double)30;
    camera_config_.trigger_source = std::string("FixedRate");
    camera_config_.trigger_mode = std::string("On");
    camera_config_.trigger_selector = std::string("FrameStart");
    camera_config_.trigger_activation = std::string("RisingEdge");
    camera_config_.trigger_delay = (double)0.0;
    camera_config_.exposure = (double)50000;
    camera_config_.exposure_auto = std::string("Continuous");
    camera_config_.exposure_auto_alg = std::string("FitRange");
    camera_config_.exposure_auto_tol = 5;
    camera_config_.exposure_auto_max = 50000;
    camera_config_.exposure_auto_min = 41;
    camera_config_.exposure_auto_outliers = 0;
    camera_config_.exposure_auto_rate = 100;
    camera_config_.exposure_auto_target = 50;
    camera_config_.gain = (double)0;
    camera_config_.gain_auto = std::string("Continuous");
    camera_config_.gain_auto_tol = 5;
    camera_config_.gain_auto_max = (double)32;
    camera_config_.gain_auto_min = (double)0;
    camera_config_.gain_auto_outliers = 0;
    camera_config_.gain_auto_rate = 100;
    camera_config_.gain_auto_target = 50;
    camera_config_.balance_ratio_abs = (double)1.0;
    camera_config_.balance_ratio_selector = std::string("Red");
    camera_config_.whitebalance_auto = std::string("Continuous");
    camera_config_.whitebalance_auto_tol = 5;
    camera_config_.whitebalance_auto_rate = 100;
    camera_config_.binning_x = 1;
    camera_config_.binning_y = 1;
    camera_config_.decimation_x = 1;
    camera_config_.decimation_y = 1;
    camera_config_.width = 2064;
    camera_config_.height = 1544;
    camera_config_.roi_width = 0;
    camera_config_.roi_height = 0;
    camera_config_.roi_offset_x = 0;
    camera_config_.roi_offset_y = 0;
    camera_config_.pixel_format = std::string("Mono8");
    camera_config_.stream_bytes_per_second = 45000000;
    camera_config_.ptp_mode = std::string("Off");
    camera_config_.sync_in_selector = std::string("SyncIn1");
    camera_config_.sync_out_polarity = std::string("Normal");
    camera_config_.sync_out_selector = std::string("SyncOut1");
    camera_config_.sync_out_source = std::string("GPO");
    camera_config_.iris_auto_target = 50;
    camera_config_.iris_mode = std::string("Continuous");
    camera_config_.iris_video_level_min = 110;
    camera_config_.iris_video_level_max = 110;


  // Start dynamic_reconfigure & run configure()
  //np reconfigure_server_.setCallback(boost::bind(&avt_vimba_camera::MonoCamera::configure, this, _1, _2));
  ip_ = std::string("192.168.1.23");  // [neil-rti] test
  configure(camera_config_, 0);

}

MonoCamera::~MonoCamera(void) {
  api_.shutdown();
  //np pub_.shutdown();
}

void MonoCamera::frameCallback(const FramePtr& vimba_frame_ptr) {
  static int nth = 20;
  if(--nth <= 0) {
    nth = 20;
  std::cout << "In the callback " << camera_info_pub_.getNumSubscribers() << std::endl;
#if 1 //np
  //np ros::Time ros_time = ros::Time::now();
  rclcpp::Time ros_time = ros_clock_.now();

  if (camera_info_pub_.getNumSubscribers() > 0) {
    sensor_msgs::msg::Image img;
    if (api_.frameToImage(vimba_frame_ptr, img)) {
      sensor_msgs::msg::CameraInfo ci = info_man_->getCameraInfo();
      ci.header.stamp = img.header.stamp = ros_time;
      img.header.frame_id = ci.header.frame_id;
      camera_info_pub_.publish(img, ci);
    } else {
      //np ROS_WARN_STREAM("Function frameToImage returned 0. No image published.");
      std::cout << "Function frameToImage returned 0. No image published." << std::endl;
    }
  }
  }
  // updater_.update();
#endif  //np
}

#if 1 //np
/** Dynamic reconfigure callback
*
*  Called immediately when callback first defined. Called again
*  when dynamic reconfigure starts or changes a parameter value.
*
*  @param newconfig new Config values
*  @param level bit-wise OR of reconfiguration levels for all
*               changed parameters (0xffffffff on initial call)
**/
void MonoCamera::configure(Config& newconfig, uint32_t level) {
  try {
    // resolve frame ID using tf_prefix parameter
    if (newconfig.frame_id == "") {
      newconfig.frame_id = "camera";
    }
    // The camera already stops & starts acquisition
    // so there's no problem on changing any feature.
    if (!cam_.isOpened()) {
      cam_.start(ip_, guid_, show_debug_prints_);
    }

    Config config = newconfig;
    cam_.updateConfig(newconfig);
    updateCameraInfo(config);
  } catch (const std::exception& e) {
    //np ROS_ERROR_STREAM("Error reconfiguring mono_camera node : " << e.what());
    std::cout << "Error reconfiguring mono_camera node : " << e.what() << std::endl;
  }
}

void MonoCamera::updateCameraInfo(const avt_vimba_camera::AvtVimbaCameraConfig& config) {

  // Get camera_info from the manager
  sensor_msgs::msg::CameraInfo ci = info_man_->getCameraInfo();

  // Set the frame id
  ci.header.frame_id = config.frame_id;

  // Set the operational parameters in CameraInfo (binning, ROI)
  int binning_or_decimation_x = std::max(config.binning_x, config.decimation_x);
  int binning_or_decimation_y = std::max(config.binning_y, config.decimation_y);

  // Set the operational parameters in CameraInfo (binning, ROI)
  ci.height    = config.height;
  ci.width     = config.width;
  ci.binning_x = binning_or_decimation_x;
  ci.binning_y = binning_or_decimation_y;

  // ROI in CameraInfo is in unbinned coordinates, need to scale up
  ci.roi.x_offset = config.roi_offset_x;
  ci.roi.y_offset = config.roi_offset_y;
  ci.roi.height   = config.roi_height;
  ci.roi.width    = config.roi_width;

  // set the new URL and load CameraInfo (if any) from it
  std::string camera_info_url;
  //np nhp_.getParam("camera_info_url", camera_info_url);
  if (camera_info_url != camera_info_url_) {
    info_man_->setCameraName(config.frame_id);
    if (info_man_->validateURL(camera_info_url)) {
      info_man_->loadCameraInfo(camera_info_url);
      ci = info_man_->getCameraInfo();
    } else {
      //np ROS_WARN_STREAM("Camera info URL not valid: " << camera_info_url);
      std::cout << "Camera info URL not valid: " << camera_info_url << std::endl;
    }
  }

  bool roiMatchesCalibration = (ci.height == config.roi_height
                              && ci.width == config.roi_width);
  bool resolutionMatchesCalibration = (ci.width == config.width
                                   && ci.height == config.height);
  // check
  ci.roi.do_rectify = roiMatchesCalibration || resolutionMatchesCalibration;

  // push the changes to manager
  info_man_->setCameraInfo(ci);
}
#endif //np
}
