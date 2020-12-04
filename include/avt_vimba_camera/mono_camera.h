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

#ifndef __MONO_CAMERA_H__
#define __MONO_CAMERA_H__

#include <avt_vimba_camera/avt_vimba_camera.h>

namespace avt_vimba_camera 
{
class MonoCamera : public rclcpp::Node {
public:
    explicit MonoCamera(const rclcpp::NodeOptions&);
    ~MonoCamera();

    rclcpp::Node::SharedPtr node_handle_;

private:
    AvtVimbaApi api_;
    AvtVimbaCamera cam_;

    std::string ip_;
    std::string guid_;
    std::string camera_info_url_;
    bool show_debug_prints_;

    // Camera configuration
    typedef avt_vimba_camera::AvtVimbaCameraConfig Config;
    Config camera_config_;

    // Camera parameters
    // typedef avt_vimba_camera::AvtVimbaCameraParms CfgParms;
    std::shared_ptr<avt_vimba_camera::AvtVimbaCameraParms> camera_parms_;

    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::steady_clock::time_point last_frame_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> info_man_;
    image_transport::CameraPublisher camera_info_pub_;
    // std::shared_ptr<sensor_msgs::msg::Image> image_msg_;
    rclcpp::Clock ros_clock_;

    void ImageCallback();


    void frameCallback(const FramePtr& vimba_frame_ptr);
    void configure(Config& newconfig, uint32_t level);
    void updateCameraInfo(const Config& config);
};
}
#endif  // __MONO_CAMERA_H__
