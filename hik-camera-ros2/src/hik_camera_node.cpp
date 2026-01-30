#include "MvCameraControl.h"
// ROS
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace hik_camera
{
class HikCameraNode : public rclcpp::Node
{
public:
  explicit HikCameraNode(const rclcpp::NodeOptions & options) : Node("hik_camera", options)
  {
    RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode!");

    MV_CC_DEVICE_INFO_LIST device_list;
    // enum device
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    RCLCPP_INFO(this->get_logger(), "Found camera count = %d", device_list.nDeviceNum);

    while (device_list.nDeviceNum == 0 && rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "No camera found!");
      RCLCPP_INFO(this->get_logger(), "Enum state: [%x]", nRet);
      std::this_thread::sleep_for(std::chrono::seconds(1));
      nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    }

    MV_CC_CreateHandle(&camera_handle_, device_list.pDeviceInfo[0]);

    MV_CC_OpenDevice(camera_handle_);

    // Get camera infomation
    MV_CC_GetImageInfo(camera_handle_, &img_info_);
    image_msg_.data.reserve(img_info_.nHeightMax * img_info_.nWidthMax * 3);

    convert_param_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;

    declareParameters();

    // Re-read image info after parameter declaration as resolution might have changed
    MV_CC_GetImageInfo(camera_handle_, &img_info_);
    image_msg_.data.reserve(img_info_.nHeightMax * img_info_.nWidthMax * 3);
    convert_param_.nWidth = img_info_.nWidthValue;
    convert_param_.nHeight = img_info_.nHeightValue;

    bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
    auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
    camera_pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);

    // publishing control parameters
    double publish_rate = this->declare_parameter("publish_rate", 10.0);
    publish_rate = std::max(0.0, publish_rate);
    this->publish_period_secs_ = (publish_rate > 0.0 ? 1.0 / publish_rate : 0.0);
    this->last_publish_time_ = this->now() - rclcpp::Duration::from_seconds(this->publish_period_secs_);

    this->suppress_get_buffer_warnings_ = this->declare_parameter("suppress_get_buffer_warnings", true);



    // declareParameters(); // Moved earlier to affect init buffers

    MV_CC_StartGrabbing(camera_handle_);

    // Load camera info
    camera_name_ = this->declare_parameter("camera_name", "narrow_stereo");
    camera_info_manager_ =
      std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);
    auto camera_info_url =
      this->declare_parameter("camera_info_url", "package://hik_camera/config/camera_info.yaml");
    if (camera_info_manager_->validateURL(camera_info_url)) {
      camera_info_manager_->loadCameraInfo(camera_info_url);
      camera_info_msg_ = camera_info_manager_->getCameraInfo();
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
    }

    params_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

    capture_thread_ = std::thread{[this]() -> void {
      MV_FRAME_OUT out_frame;

      RCLCPP_INFO(this->get_logger(), "Publishing image!");

      image_msg_.header.frame_id = "camera_optical_frame";
      image_msg_.encoding = "rgb8";

      while (rclcpp::ok()) {
        nRet = MV_CC_GetImageBuffer(camera_handle_, &out_frame, 1000);
        if (MV_OK == nRet) {
          // ensure destination buffer is allocated before conversion
          image_msg_.height = out_frame.stFrameInfo.nHeight;
          image_msg_.width = out_frame.stFrameInfo.nWidth;
          image_msg_.step = out_frame.stFrameInfo.nWidth * 3;
          image_msg_.data.resize(image_msg_.width * image_msg_.height * 3);

          convert_param_.pDstBuffer = image_msg_.data.data();
          convert_param_.nDstBufferSize = static_cast<int>(image_msg_.data.size());
          convert_param_.pSrcData = out_frame.pBufAddr;
          convert_param_.nSrcDataLen = out_frame.stFrameInfo.nFrameLen;
          convert_param_.enSrcPixelType = out_frame.stFrameInfo.enPixelType;

          MV_CC_ConvertPixelType(camera_handle_, &convert_param_);

          // Throttle publishing according to publish_rate (0 = unlimited)
          rclcpp::Time now = this->now();
          if (this->publish_period_secs_ <= 0.0 || (now - this->last_publish_time_).seconds() >= this->publish_period_secs_) {
            image_msg_.header.stamp = now;
            camera_info_msg_.header = image_msg_.header;
            camera_pub_.publish(image_msg_, camera_info_msg_);
            this->last_publish_time_ = now;
          } else {
            // skip publishing this frame to control rate
          }

          MV_CC_FreeImageBuffer(camera_handle_, &out_frame);
          fail_conut_ = 0;
        } else {
          if (this->suppress_get_buffer_warnings_) {
            RCLCPP_DEBUG(this->get_logger(), "Get buffer failed! nRet: [%x]", nRet);
          } else {
            RCLCPP_WARN(this->get_logger(), "Get buffer failed! nRet: [%x]", nRet);
          }
          MV_CC_StopGrabbing(camera_handle_);
          MV_CC_StartGrabbing(camera_handle_);
          fail_conut_++;
        }

        if (fail_conut_ > 5) {
          RCLCPP_FATAL(this->get_logger(), "Camera failed!");
          rclcpp::shutdown();
        }
      }
    }};
  }

  ~HikCameraNode() override
  {
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    if (camera_handle_) {
      MV_CC_StopGrabbing(camera_handle_);
      MV_CC_CloseDevice(camera_handle_);
      MV_CC_DestroyHandle(&camera_handle_);
    }
    RCLCPP_INFO(this->get_logger(), "HikCameraNode destroyed!");
  }

private:
  double publish_period_secs_ = 0.0;
  rclcpp::Time last_publish_time_;
  bool suppress_get_buffer_warnings_ = true;
  void declareParameters()
  {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    MVCC_FLOATVALUE f_value;
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;
    // Exposure time
    param_desc.description = "Exposure time in microseconds";
    MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &f_value);
    param_desc.integer_range[0].from_value = f_value.fMin;
    param_desc.integer_range[0].to_value = f_value.fMax;
    double exposure_time = this->declare_parameter("exposure_time", 5000, param_desc);
    MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);
    RCLCPP_INFO(this->get_logger(), "Exposure time: %f", exposure_time);

    // Gain
    param_desc.description = "Gain";
    MV_CC_GetFloatValue(camera_handle_, "Gain", &f_value);
    param_desc.integer_range[0].from_value = f_value.fMin;
    param_desc.integer_range[0].to_value = f_value.fMax;
    double gain = this->declare_parameter("gain", f_value.fCurValue, param_desc);
    MV_CC_SetFloatValue(camera_handle_, "Gain", gain);
    RCLCPP_INFO(this->get_logger(), "Gain: %f", gain);

    // Acquisition Frame Rate
    // Attempt to enable frame rate control if available
    MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", true);
    
    param_desc.description = "Acquisition Frame Rate in Hz";
    MV_CC_GetFloatValue(camera_handle_, "AcquisitionFrameRate", &f_value);
    param_desc.integer_range[0].from_value = f_value.fMin;
    param_desc.integer_range[0].to_value = f_value.fMax;
    // Default to read value or a reasonable default if read fails, user can override
    double current_rate = f_value.fCurValue;
    double acquisition_frame_rate = this->declare_parameter("acquisition_frame_rate", current_rate, param_desc);
    
    // Apply the parameter value
    nRet = MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", acquisition_frame_rate);
    if (MV_OK != nRet) {
      RCLCPP_WARN(this->get_logger(), "Failed to set AcquisitionFrameRate to %f, nRet: [%x]", acquisition_frame_rate, nRet);
    } else {
        RCLCPP_INFO(this->get_logger(), "Acquisition Frame Rate: %f", acquisition_frame_rate);
    }

    // Resolution Control
    MVCC_INTVALUE_EX tWidth, tHeight, tOffsetX, tOffsetY;
    MV_CC_GetIntValueEx(camera_handle_, "Width", &tWidth);
    MV_CC_GetIntValueEx(camera_handle_, "Height", &tHeight);
    MV_CC_GetIntValueEx(camera_handle_, "OffsetX", &tOffsetX);
    MV_CC_GetIntValueEx(camera_handle_, "OffsetY", &tOffsetY);

    int width = this->declare_parameter("image_width", (int)tWidth.nCurValue);
    int height = this->declare_parameter("image_height", (int)tHeight.nCurValue);
    int offset_x = this->declare_parameter("offset_x", (int)tOffsetX.nCurValue);
    int offset_y = this->declare_parameter("offset_y", (int)tOffsetY.nCurValue);

    // Set Offset first to 0 to avoid "Offset + Width > Max" error when sizing up
    MV_CC_SetIntValueEx(camera_handle_, "OffsetX", 0);
    MV_CC_SetIntValueEx(camera_handle_, "OffsetY", 0);
    
    // Set Size
    nRet = MV_CC_SetIntValueEx(camera_handle_, "Width", width);
    if (nRet != MV_OK) RCLCPP_WARN(this->get_logger(), "Failed to set Width to %d, nRet: %x", width, nRet);

    nRet = MV_CC_SetIntValueEx(camera_handle_, "Height", height);
    if (nRet != MV_OK) RCLCPP_WARN(this->get_logger(), "Failed to set Height to %d, nRet: %x", height, nRet);

    // Set Offset back
    nRet = MV_CC_SetIntValueEx(camera_handle_, "OffsetX", offset_x);
    if (nRet != MV_OK) RCLCPP_WARN(this->get_logger(), "Failed to set OffsetX to %d, nRet: %x", offset_x, nRet);

    nRet = MV_CC_SetIntValueEx(camera_handle_, "OffsetY", offset_y);
    if (nRet != MV_OK) RCLCPP_WARN(this->get_logger(), "Failed to set OffsetY to %d, nRet: %x", offset_y, nRet);

    RCLCPP_INFO(this->get_logger(), "ROI Set: %dx%d @ (%d,%d)", width, height, offset_x, offset_y);
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & param : parameters) {
      if (param.get_name() == "exposure_time") {
        int status = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_int());
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set exposure time, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "gain") {
        int status = MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set gain, status = " + std::to_string(status);
        }
      } else if (param.get_name() == "acquisition_frame_rate") {
        // Ensure control is enabled
        MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", true);
        
        int status = MV_CC_SetFloatValue(camera_handle_, "AcquisitionFrameRate", param.as_double());
        if (MV_OK != status) {
          result.successful = false;
          result.reason = "Failed to set acquisition_frame_rate, status = " + std::to_string(status);
        } else {
           RCLCPP_INFO(this->get_logger(), "Set AcquisitionFrameRate: %f", param.as_double());
        }
      } else if (param.get_name() == "image_width") {
        int status = MV_CC_SetIntValueEx(camera_handle_, "Width", param.as_int());
        if (MV_OK != status) { result.successful = false; result.reason = "Failed to set width, status = " + std::to_string(status); }
      } else if (param.get_name() == "image_height") {
        int status = MV_CC_SetIntValueEx(camera_handle_, "Height", param.as_int());
        if (MV_OK != status) { result.successful = false; result.reason = "Failed to set height, status = " + std::to_string(status); }
      } else if (param.get_name() == "offset_x") {
        int status = MV_CC_SetIntValueEx(camera_handle_, "OffsetX", param.as_int());
        if (MV_OK != status) { result.successful = false; result.reason = "Failed to set offset_x, status = " + std::to_string(status); }
      } else if (param.get_name() == "offset_y") {
        int status = MV_CC_SetIntValueEx(camera_handle_, "OffsetY", param.as_int());
        if (MV_OK != status) { result.successful = false; result.reason = "Failed to set offset_y, status = " + std::to_string(status); }
      } else {
        result.successful = false;
        result.reason = "Unknown parameter: " + param.get_name();
      }
    }
    return result;
  }

  sensor_msgs::msg::Image image_msg_;

  image_transport::CameraPublisher camera_pub_;

  int nRet = MV_OK;
  void * camera_handle_;
  MV_IMAGE_BASIC_INFO img_info_;

  MV_CC_PIXEL_CONVERT_PARAM convert_param_;

  std::string camera_name_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  sensor_msgs::msg::CameraInfo camera_info_msg_;

  int fail_conut_ = 0;
  std::thread capture_thread_;

  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
};
}  // namespace hik_camera

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(hik_camera::HikCameraNode)
