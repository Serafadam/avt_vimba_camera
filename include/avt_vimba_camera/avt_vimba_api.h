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

#ifndef AVT_VIMBA_API_H
#define AVT_VIMBA_API_H

#include <VimbaCPP/Include/VimbaCPP.h>

//#include <ros/ros.h>
//#include <ros/console.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/fill_image.hpp"

#include <string>
#include <map>

using AVT::VmbAPI::VimbaSystem;
using AVT::VmbAPI::CameraPtr;
using AVT::VmbAPI::FramePtr;
using AVT::VmbAPI::CameraPtrVector;

namespace avt_vimba_camera {
class AvtVimbaApi {
 public:
  AvtVimbaApi(void) : vs(VimbaSystem::GetInstance()) {}

  void start() {
    VmbErrorType err = vs.Startup();
    if (VmbErrorSuccess == err) {
      //np RCLCPP_ERROR_STREAM("[Vimba System]: AVT Vimba System initialized successfully");      
      std::cout << "[Vimba System]: AVT Vimba System initialized successfully" << std::endl;
      listAvailableCameras();
    } else {
      //np RCLCPP_ERROR_STREAM("[Vimba System]: Could not start Vimba system: " << errorCodeToMessage(err));
      std::cout << "[Vimba System]: Could not start Vimba system: " << errorCodeToMessage(err) << std::endl;
    }
  }

  // [neil-rti] add a shutdown method, why not?
  void shutdown() {
    VmbErrorType err = vs.Shutdown();
    if (VmbErrorSuccess == err) {
      std::cout << "[Vimba System]: AVT Vimba System shut down successfully" << std::endl;
    } else {
      std::cout << "[Vimba System]: Could not shut down Vimba system: " << errorCodeToMessage(err) << std::endl;
    }

  }

  /** Translates Vimba error codes to readable error messages
    *
    * @param error Vimba error tyme
    * @return readable string error
    *
    **/
  std::string errorCodeToMessage(VmbErrorType error) {
    std::map<VmbErrorType, std::string> error_msg;
    error_msg[VmbErrorSuccess]        = "Success.";
    error_msg[VmbErrorApiNotStarted]  = "API not started.";
    error_msg[VmbErrorNotFound]       = "Not found.";
    error_msg[VmbErrorBadHandle]      = "Invalid handle ";
    error_msg[VmbErrorDeviceNotOpen]  = "Device not open.";
    error_msg[VmbErrorInvalidAccess]  = "Invalid access.";
    error_msg[VmbErrorBadParameter]   = "Bad parameter.";
    error_msg[VmbErrorStructSize]     = "Wrong DLL version.";
    error_msg[VmbErrorWrongType]      = "Wrong type.";
    error_msg[VmbErrorInvalidValue]   = "Invalid value.";
    error_msg[VmbErrorTimeout]        = "Timeout.";
    error_msg[VmbErrorOther]          = "TL error.";
    error_msg[VmbErrorInvalidCall]    = "Invalid call.";
    error_msg[VmbErrorNoTL]           = "TL not loaded.";
    error_msg[VmbErrorNotImplemented] = "Not implemented.";
    error_msg[VmbErrorNotSupported]   = "Not supported.";
    error_msg[VmbErrorResources]      = "Resource not available.";
    error_msg[VmbErrorInternalFault]  = "Unexpected fault in VmbApi or driver.";
    error_msg[VmbErrorMoreData]       = "More data returned than memory provided.";

    std::map<VmbErrorType, std::string>::const_iterator iter =
      error_msg.find(error);
    if ( error_msg.end() != iter ) {
      return iter->second;
    }
    return "Unsupported error code passed.";
  }

  bool frameToImage(const FramePtr vimba_frame_ptr, sensor_msgs::msg::Image& image) {
    VmbPixelFormatType pixel_format;
    VmbUint32_t width, height, nSize;

    vimba_frame_ptr->GetWidth(width);
    vimba_frame_ptr->GetHeight(height);
    vimba_frame_ptr->GetPixelFormat(pixel_format);
    vimba_frame_ptr->GetImageSize(nSize);

    VmbUint32_t step = nSize / height;

    // NOTE: YUV and ARGB formats not supported
    std::string encoding;
    if      (pixel_format == VmbPixelFormatMono8          ) encoding = sensor_msgs::image_encodings::MONO8;
    else if (pixel_format == VmbPixelFormatMono10         ) encoding = sensor_msgs::image_encodings::MONO16;
    else if (pixel_format == VmbPixelFormatMono12         ) encoding = sensor_msgs::image_encodings::MONO16;
    else if (pixel_format == VmbPixelFormatMono12Packed   ) encoding = sensor_msgs::image_encodings::MONO16;
    else if (pixel_format == VmbPixelFormatMono14         ) encoding = sensor_msgs::image_encodings::MONO16;
    else if (pixel_format == VmbPixelFormatMono16         ) encoding = sensor_msgs::image_encodings::MONO16;
    else if (pixel_format == VmbPixelFormatBayerGR8       ) encoding = sensor_msgs::image_encodings::BAYER_GRBG8;
    else if (pixel_format == VmbPixelFormatBayerRG8       ) encoding = sensor_msgs::image_encodings::BAYER_RGGB8;
    else if (pixel_format == VmbPixelFormatBayerGB8       ) encoding = sensor_msgs::image_encodings::BAYER_GBRG8;
    else if (pixel_format == VmbPixelFormatBayerBG8       ) encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
    else if (pixel_format == VmbPixelFormatBayerGR10      ) encoding = sensor_msgs::image_encodings::TYPE_16SC1;
    else if (pixel_format == VmbPixelFormatBayerRG10      ) encoding = sensor_msgs::image_encodings::TYPE_16SC1;
    else if (pixel_format == VmbPixelFormatBayerGB10      ) encoding = sensor_msgs::image_encodings::TYPE_16SC1;
    else if (pixel_format == VmbPixelFormatBayerBG10      ) encoding = sensor_msgs::image_encodings::TYPE_16SC1;
    else if (pixel_format == VmbPixelFormatBayerGR12      ) encoding = sensor_msgs::image_encodings::TYPE_16SC1;
    else if (pixel_format == VmbPixelFormatBayerRG12      ) encoding = sensor_msgs::image_encodings::TYPE_16SC1;
    else if (pixel_format == VmbPixelFormatBayerGB12      ) encoding = sensor_msgs::image_encodings::TYPE_16SC1;
    else if (pixel_format == VmbPixelFormatBayerBG12      ) encoding = sensor_msgs::image_encodings::TYPE_16SC1;
    else if (pixel_format == VmbPixelFormatBayerGR12Packed) encoding = sensor_msgs::image_encodings::TYPE_32SC4;
    else if (pixel_format == VmbPixelFormatBayerRG12Packed) encoding = sensor_msgs::image_encodings::TYPE_32SC4;
    else if (pixel_format == VmbPixelFormatBayerGB12Packed) encoding = sensor_msgs::image_encodings::TYPE_32SC4;
    else if (pixel_format == VmbPixelFormatBayerBG12Packed) encoding = sensor_msgs::image_encodings::TYPE_32SC4;
    else if (pixel_format == VmbPixelFormatBayerGR16      ) encoding = sensor_msgs::image_encodings::TYPE_16SC1;
    else if (pixel_format == VmbPixelFormatBayerRG16      ) encoding = sensor_msgs::image_encodings::TYPE_16SC1;
    else if (pixel_format == VmbPixelFormatBayerGB16      ) encoding = sensor_msgs::image_encodings::TYPE_16SC1;
    else if (pixel_format == VmbPixelFormatBayerBG16      ) encoding = sensor_msgs::image_encodings::TYPE_16SC1;
    else if (pixel_format == VmbPixelFormatRgb8           ) encoding = sensor_msgs::image_encodings::RGB8;
    else if (pixel_format == VmbPixelFormatBgr8           ) encoding = sensor_msgs::image_encodings::BGR8;
    else if (pixel_format == VmbPixelFormatRgba8          ) encoding = sensor_msgs::image_encodings::RGBA8;
    else if (pixel_format == VmbPixelFormatBgra8          ) encoding = sensor_msgs::image_encodings::BGRA8;
    else if (pixel_format == VmbPixelFormatRgb12          ) encoding = sensor_msgs::image_encodings::TYPE_16UC3;
    else if (pixel_format == VmbPixelFormatRgb16          ) encoding = sensor_msgs::image_encodings::TYPE_16UC3;
    else {
      //np RCLCPP_WARN_ONCE(rclcpp::Node::get_logger(), "Received frame with unsupported pixel format %d", pixel_format);
      std::cout << "Received frame with unsupported pixel format " << pixel_format << std::endl;
    }
    if (encoding == "") return false;

    VmbUchar_t *buffer_ptr;
    VmbErrorType err = vimba_frame_ptr->GetImage(buffer_ptr);
    bool res = false;
    if ( VmbErrorSuccess == err ) {
      res = sensor_msgs::fillImage(image,
                                   encoding,
                                   height,
                                   width,
                                   step,
                                   buffer_ptr);
    } else {
#if 0 //np
      ROS_ERROR_STREAM("[" << ros::this_node::getName()
        << "]: Could not GetImage. "
        << "\n Error: " << errorCodeToMessage(err));
#else
      std::cout << "[" << "rclcpp::Node::get_name()"
        << "]: Could not GetImage. "
        << "\n Error: " << errorCodeToMessage(err) << std::endl;
#endif  //np
    }
    return res;
  }

 private:
  VimbaSystem& vs;

  void listAvailableCameras(void) {
    std::string name;
    CameraPtrVector cameras;
    if (VmbErrorSuccess == vs.Startup()) {
      if (VmbErrorSuccess == vs.GetCameras(cameras)) {
        for (CameraPtrVector::iterator iter = cameras.begin();
        cameras.end() != iter;
        ++iter) {
          if (VmbErrorSuccess == (*iter)->GetName(name)) {
#if 0 //np
            ROS_DEBUG_STREAM("[" << ros::this_node::getName() << "]: Found camera: ");
#else
            std::cout << "[" << "rclcpp::Node::get_name()" << "]: Found camera: " << std::endl;
#endif  //np
          }
          std::string strID;            // The ID of the cam
          std::string strName;          // The name of the cam
          std::string strModelname;     // The model name of the cam
          std::string strSerialNumber;  // The serial number of the cam
          std::string strInterfaceID;  // The ID of the interface the cam is connected to
          VmbErrorType err = (*iter)->GetID( strID );
          if ( VmbErrorSuccess != err )
          {
#if 0 //np
              ROS_ERROR_STREAM("[Could not get camera ID. Error code: " << err << "]");
#else
              std::cout << "[Could not get camera ID. Error code: " << err << "]" << std::endl;
#endif  //np
          }
          err = (*iter)->GetName( strName );
          if ( VmbErrorSuccess != err )
          {
#if 0 //np
              ROS_ERROR_STREAM("[Could not get camera name. Error code: " << err << "]");
#else
              std::cout << "[Could not get camera name. Error code: " << err << "]" << std::endl;
#endif  //np
          }

          err = (*iter)->GetModel( strModelname );
          if ( VmbErrorSuccess != err )
          {
#if 0 //np
              ROS_ERROR_STREAM("[Could not get camera mode name. Error code: " << err << "]");
#else
              std::cout << "[Could not get camera mode name. Error code: " << err << "]" << std::endl;
#endif  //np
          }

          err = (*iter)->GetSerialNumber( strSerialNumber );
          if ( VmbErrorSuccess != err )
          {
#if 0 //np
              ROS_ERROR_STREAM("[Could not get camera serial number. Error code: " << err << "]");
#else
              std::cout << "[Could not get camera serial number. Error code: " << err << "]" << std::endl;
#endif  //np
          }

          err = (*iter)->GetInterfaceID( strInterfaceID );
          if ( VmbErrorSuccess != err )
          {
#if 0 //np
              ROS_ERROR_STREAM("[Could not get interface ID. Error code: " << err << "]");
#else
              std::cout << "[Could not get interface ID. Error code: " << err << "]" << std::endl;
#endif  //np
          }
#if 0 //np
          ROS_INFO_STREAM("\t/// Camera Name: " << strName);
          ROS_INFO_STREAM("\t/// Model Name: " << strModelname);
          ROS_INFO_STREAM("\t/// Camera ID: " << strID);
          ROS_INFO_STREAM("\t/// Serial Number: " << strSerialNumber);
          ROS_INFO_STREAM("\t/// @ Interface ID: " << strInterfaceID);
#else
          std::cout << "\t/// Camera Name: " << strName << std::endl;
          std::cout << "\t/// Model Name: " << strModelname << std::endl;
          std::cout << "\t/// Camera ID: " << strID << std::endl;
          std::cout << "\t/// Serial Number: " << strSerialNumber << std::endl;
          std::cout << "\t/// @ Interface ID: " << strInterfaceID << std::endl;
#endif  //np

        }
      } else {
#if 0 //np
        ROS_WARN("Could not get cameras from Vimba System");
#else
        std::cout << "Could not get cameras from Vimba System" << std::endl;
#endif  //np
      }
    } else {
#if 0 //np
      ROS_WARN("Could not start Vimba System");
#else
      std::cout << "Could not start Vimba System" << std::endl;
#endif  //np
    }
  }
};
}
#endif
