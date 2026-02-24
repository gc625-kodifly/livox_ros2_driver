//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include <chrono>
#include <cstring>
#include <fstream>
#include <future>
#include <iomanip>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "include/livox_ros2_driver.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "lddc.h"
#include "lds_hub.h"
#include "lds_lidar.h"
#include "lds_lvx.h"
#include "livox_sdk.h"
#include "livox_interfaces/msg/custom_point.h"
#include "livox_interfaces/msg/custom_msg.h"

namespace
{
  const int32_t kSdkVersionMajorLimit = 2;
}

namespace livox_ros
{

LivoxDriver::LivoxDriver()
  : LifecycleNode("livox_driver_node")
{
  RCLCPP_INFO(this->get_logger(), "Livox Ros Driver Version: %s",
    LIVOX_ROS_DRIVER_VERSION_STRING);
  declareParameters();
}

LivoxDriver::~LivoxDriver()
{
  try {
    exit_signal_.set_value();
  } catch (...) {}
  if (poll_thread_ && poll_thread_->joinable()) {
    poll_thread_->join();
  }
}

void LivoxDriver::declareParameters()
{
  this->declare_parameter("xfer_format", 1);
  this->declare_parameter("multi_topic", 0);
  this->declare_parameter("data_src", 0);
  this->declare_parameter("publish_freq", 10.0);
  this->declare_parameter("output_data_type", 0);
  this->declare_parameter("frame_id", std::string("livox_frame"));
  this->declare_parameter("cmdline_input_bd_code", std::string("livox0000000001"));
  this->declare_parameter("lvx_file_path", std::string("/home/livox/livox_test.lvx"));

  this->declare_parameter("num_lidars", 1);
  int num_lidars = this->get_parameter("num_lidars").as_int();
  for (int i = 0; i < num_lidars; i++) {
    std::string p = "lidar" + std::to_string(i) + "_";
    this->declare_parameter(p + "broadcast_code", std::string(""));
    this->declare_parameter(p + "enable_connect", false);
    this->declare_parameter(p + "enable_fan", true);
    this->declare_parameter(p + "return_mode", 0);
    this->declare_parameter(p + "coordinate", 0);
    this->declare_parameter(p + "imu_rate", 1);
    this->declare_parameter(p + "extrinsic_parameter_source", 0);
    this->declare_parameter(p + "enable_high_sensitivity", false);
  }

  this->declare_parameter("enable_timesync", false);
  this->declare_parameter("timesync_device_name", std::string("/dev/ttyUSB0"));
  this->declare_parameter("timesync_comm_device_type", 0);
  this->declare_parameter("timesync_baudrate_index", 2);
  this->declare_parameter("timesync_parity_index", 0);

  this->declare_parameter("config_file_path", std::string(""));
}

void LivoxDriver::readParameters()
{
  this->get_parameter("xfer_format", xfer_format_);
  this->get_parameter("multi_topic", multi_topic_);
  this->get_parameter("data_src", data_src_);
  this->get_parameter("publish_freq", publish_freq_);
  this->get_parameter("output_data_type", output_type_);
  this->get_parameter("frame_id", frame_id_);
  this->get_parameter("cmdline_input_bd_code", cmdline_bd_code_);
  this->get_parameter("lvx_file_path", lvx_file_path_);
  this->get_parameter("num_lidars", num_lidars_);
  this->get_parameter("enable_timesync", enable_timesync_);

  if (publish_freq_ > 100.0) {
    publish_freq_ = 100.0;
  } else if (publish_freq_ < 0.1) {
    publish_freq_ = 0.1;
  }
}

void LivoxDriver::writeCurrentConfig()
{
  std::string config_file_path;
  this->get_parameter("config_file_path", config_file_path);
  if (config_file_path.empty()) return;

  std::ofstream ofs(config_file_path);
  if (!ofs.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open config file for writing: %s",
                 config_file_path.c_str());
    return;
  }

  ofs << "livox_lidar_publisher:\n";
  ofs << "  ros__parameters:\n";

  auto param_names = this->list_parameters({}, 0).names;
  auto parameters = this->get_parameters(param_names);
  for (const auto &param : parameters) {
    if (param.get_name() == "config_file_path" || param.get_name() == "use_sim_time") {
      continue;
    }
    ofs << "    " << param.get_name() << ": ";
    switch (param.get_type()) {
      case rclcpp::ParameterType::PARAMETER_BOOL:
        ofs << (param.as_bool() ? "true" : "false") << "\n";
        break;
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        ofs << param.as_int() << "\n";
        break;
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
        ofs << std::fixed << std::setprecision(6) << param.as_double() << "\n";
        break;
      case rclcpp::ParameterType::PARAMETER_STRING:
        ofs << "\"" << param.as_string() << "\"\n";
        break;
      default:
        ofs << "null\n";
        break;
    }
  }
  ofs.close();
  RCLCPP_INFO(this->get_logger(), "Parameters written to %s", config_file_path.c_str());
}

LifecycleCallbackReturn LivoxDriver::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_configure: Configuring livox driver...");

  LivoxSdkVersion _sdkversion;
  GetLivoxSdkVersion(&_sdkversion);
  if (_sdkversion.major < kSdkVersionMajorLimit) {
    RCLCPP_ERROR(this->get_logger(),
      "The SDK version[%d.%d.%d] is too low", _sdkversion.major,
      _sdkversion.minor, _sdkversion.patch);
    return LifecycleCallbackReturn::FAILURE;
  }

  readParameters();
  writeCurrentConfig();

  lddc_ptr_ = std::make_unique<Lddc>(xfer_format_, multi_topic_, data_src_,
                                     output_type_, publish_freq_, frame_id_);
  lddc_ptr_->SetRosNode(this);

  int ret = 0;
  if (data_src_ == kSourceRawLidar) {
    RCLCPP_INFO(this->get_logger(), "Data Source is raw lidar.");

    std::vector<std::string> bd_code_list;
    ParseCommandlineInputBdCode(cmdline_bd_code_.c_str(), bd_code_list);

    std::vector<UserRawConfig> lidar_configs;
    for (int i = 0; i < num_lidars_; i++) {
      std::string p = "lidar" + std::to_string(i) + "_";
      UserRawConfig config;
      memset(&config, 0, sizeof(config));

      std::string bc = this->get_parameter(p + "broadcast_code").as_string();
      std::strncpy(config.broadcast_code, bc.c_str(), sizeof(config.broadcast_code) - 1);
      config.enable_connect = this->get_parameter(p + "enable_connect").as_bool();
      config.enable_fan = this->get_parameter(p + "enable_fan").as_bool();
      config.return_mode = static_cast<uint32_t>(this->get_parameter(p + "return_mode").as_int());
      config.coordinate = static_cast<uint32_t>(this->get_parameter(p + "coordinate").as_int());
      config.imu_rate = static_cast<uint32_t>(this->get_parameter(p + "imu_rate").as_int());
      config.extrinsic_parameter_source = static_cast<uint32_t>(
          this->get_parameter(p + "extrinsic_parameter_source").as_int());
      config.enable_high_sensitivity = this->get_parameter(p + "enable_high_sensitivity").as_bool();

      RCLCPP_INFO(this->get_logger(), "Lidar[%d] broadcast_code[%s] connect[%d] fan[%d] "
                  "return_mode[%d] coordinate[%d] imu_rate[%d]",
                  i, config.broadcast_code, config.enable_connect,
                  config.enable_fan, config.return_mode,
                  config.coordinate, config.imu_rate);

      lidar_configs.push_back(config);
    }

    TimeSyncConfig ts_config;
    memset(&ts_config, 0, sizeof(ts_config));
    std::string ts_dev = this->get_parameter("timesync_device_name").as_string();
    std::strncpy(ts_config.dev_config.name, ts_dev.c_str(),
                 sizeof(ts_config.dev_config.name) - 1);
    ts_config.dev_config.type = static_cast<uint8_t>(
        this->get_parameter("timesync_comm_device_type").as_int());
    ts_config.dev_config.config.uart.baudrate = static_cast<uint8_t>(
        this->get_parameter("timesync_baudrate_index").as_int());
    ts_config.dev_config.config.uart.parity = static_cast<uint8_t>(
        this->get_parameter("timesync_parity_index").as_int());

    LdsLidar *read_lidar = LdsLidar::GetInstance(1000 / publish_freq_);
    read_lidar->SetLogger(this->get_logger());
    lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));
    ret = read_lidar->InitLdsLidar(bd_code_list, lidar_configs,
                                   enable_timesync_, ts_config);
    if (!ret) {
      RCLCPP_INFO(this->get_logger(), "Init lds lidar success!");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Init lds lidar fail!");
      return LifecycleCallbackReturn::FAILURE;
    }
  } else if (data_src_ == kSourceRawHub) {
    RCLCPP_WARN(this->get_logger(), "Hub mode not yet supported in lifecycle driver");
    return LifecycleCallbackReturn::FAILURE;
  } else {
    RCLCPP_WARN(this->get_logger(), "LVX file mode not yet supported in lifecycle driver");
    return LifecycleCallbackReturn::FAILURE;
  }

  RCLCPP_INFO(this->get_logger(), "on_configure complete. Node is INACTIVE.");
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn LivoxDriver::on_activate(const rclcpp_lifecycle::State &previous_state)
{
  RCLCPP_INFO(this->get_logger(), "on_activate: Starting data distribution...");

  exit_signal_ = std::promise<void>();
  future_ = exit_signal_.get_future();
  poll_thread_ = std::make_shared<std::thread>(&LivoxDriver::pollThread, this);

  rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);

  RCLCPP_INFO(this->get_logger(), "on_activate: Livox driver is now ACTIVE and publishing.");
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn LivoxDriver::on_deactivate(const rclcpp_lifecycle::State &previous_state)
{
  RCLCPP_INFO(this->get_logger(), "on_deactivate: Stopping data distribution...");

  try {
    exit_signal_.set_value();
  } catch (...) {}

  if (poll_thread_ && poll_thread_->joinable()) {
    poll_thread_->join();
  }
  poll_thread_.reset();

  rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);

  RCLCPP_INFO(this->get_logger(), "on_deactivate: Livox driver is now INACTIVE.");
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn LivoxDriver::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_cleanup: Cleaning up resources...");

  if (lddc_ptr_) {
    lddc_ptr_->PrepareExit();
    lddc_ptr_.reset();
  }

  RCLCPP_INFO(this->get_logger(), "on_cleanup: Cleanup complete. Node is UNCONFIGURED.");
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn LivoxDriver::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_shutdown: Shutting down...");

  try {
    exit_signal_.set_value();
  } catch (...) {}

  if (poll_thread_ && poll_thread_->joinable()) {
    poll_thread_->join();
  }
  poll_thread_.reset();

  if (lddc_ptr_) {
    lddc_ptr_->PrepareExit();
    lddc_ptr_.reset();
  }

  RCLCPP_INFO(this->get_logger(), "on_shutdown: Shutdown complete.");
  return LifecycleCallbackReturn::SUCCESS;
}

LifecycleCallbackReturn LivoxDriver::on_error(const rclcpp_lifecycle::State &)
{
  RCLCPP_ERROR(this->get_logger(), "on_error: Error occurred!");

  try {
    exit_signal_.set_value();
  } catch (...) {}

  if (poll_thread_ && poll_thread_->joinable()) {
    poll_thread_->join();
  }
  poll_thread_.reset();

  if (lddc_ptr_) {
    lddc_ptr_->PrepareExit();
    lddc_ptr_.reset();
  }

  return LifecycleCallbackReturn::SUCCESS;
}

void LivoxDriver::pollThread()
{
  std::future_status status;
  do {
    lddc_ptr_->DistributeLidarData();
    status = future_.wait_for(std::chrono::seconds(0));
  } while (status == std::future_status::timeout);
}

}  // namespace livox_ros

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<livox_ros::LivoxDriver>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
