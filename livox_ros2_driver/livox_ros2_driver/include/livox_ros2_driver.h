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

#ifndef LIVOX_ROS2_DRIVER_INClUDE_LIVOX_ROS2_DRIVER_H_
#define LIVOX_ROS2_DRIVER_INClUDE_LIVOX_ROS2_DRIVER_H_

#define LIVOX_ROS_DRIVER_VER_MAJOR 0
#define LIVOX_ROS_DRIVER_VER_MINOR 0
#define LIVOX_ROS_DRIVER_VER_PATCH 1

#define GET_STRING(n) GET_STRING_DIRECT(n)
#define GET_STRING_DIRECT(n) #n

#define LIVOX_ROS_DRIVER_VERSION_STRING                      \
  GET_STRING(LIVOX_ROS_DRIVER_VER_MAJOR)                     \
  "." GET_STRING(LIVOX_ROS_DRIVER_VER_MINOR) "." GET_STRING( \
      LIVOX_ROS_DRIVER_VER_PATCH)

#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "lddc.h"

namespace livox_ros
{

using LifecycleCallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class LivoxDriver : public rclcpp_lifecycle::LifecycleNode
{
public:
  LivoxDriver();
  ~LivoxDriver();

  LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State &state);
  LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State &state);
  LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State &state);
  LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State &state);
  LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State &state);
  LifecycleCallbackReturn on_error(const rclcpp_lifecycle::State &state);

private:
  void declareParameters();
  void readParameters();
  void writeCurrentConfig();
  void pollThread();

  std::unique_ptr<Lddc> lddc_ptr_;
  std::shared_ptr<std::thread> poll_thread_;
  std::shared_future<void> future_;
  std::promise<void> exit_signal_;

  int xfer_format_;
  int multi_topic_;
  int data_src_;
  double publish_freq_;
  int output_type_;
  std::string frame_id_;
  std::string cmdline_bd_code_;
  std::string lvx_file_path_;
  int num_lidars_;
  bool enable_timesync_;
};

}  // namespace livox_ros

#endif  // LIVOX_ROS2_DRIVER_INClUDE_LIVOX_ROS2_DRIVER_H_
