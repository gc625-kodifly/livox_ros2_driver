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

#include "lds_lidar.h"

#include <stdio.h>
#include <string.h>
#include <memory>
#include <mutex>
#include <thread>

using namespace std;

namespace livox_ros {

/** For callback use only */
LdsLidar *g_lds_ldiar = nullptr;

/** Lds lidar function -------------------------------------------------------*/
LdsLidar::LdsLidar(uint32_t interval_ms) : Lds(interval_ms, kSourceRawLidar) {
  auto_connect_mode_ = true;
  is_initialized_ = false;

  whitelist_count_ = 0;
  memset(broadcast_code_whitelist_, 0, sizeof(broadcast_code_whitelist_));

  ResetLdsLidar();
}

LdsLidar::~LdsLidar() {}

void LdsLidar::ResetLdsLidar(void) { ResetLds(kSourceRawLidar); }

int LdsLidar::InitLdsLidar(std::vector<std::string> &broadcast_code_strs,
                           const std::vector<UserRawConfig> &lidar_configs,
                           bool enable_timesync,
                           const TimeSyncConfig &ts_config) {
  if (is_initialized_) {
    RCLCPP_WARN(logger_, "LiDAR data source is already initialized");
    return -1;
  }

  DisableConsoleLogger();

  if (!Init()) {
    Uninit();
    RCLCPP_ERROR(logger_, "Livox-SDK init fail");
    return -1;
  }

  LivoxSdkVersion _sdkversion;
  GetLivoxSdkVersion(&_sdkversion);
  RCLCPP_INFO(logger_, "Livox SDK version %d.%d.%d",
              _sdkversion.major, _sdkversion.minor, _sdkversion.patch);

  SetBroadcastCallback(OnDeviceBroadcast);
  SetDeviceStateUpdateCallback(OnDeviceChange);

  /** Add commandline input broadcast code */
  for (auto &input_str : broadcast_code_strs) {
    AddBroadcastCodeToWhitelist(input_str.c_str());
  }

  /** Add lidar configs from ROS parameters (replaces JSON ParseConfigFile) */
  for (const auto &cfg : lidar_configs) {
    RCLCPP_INFO(logger_, "Lidar config: broadcast_code[%s] connect[%d] fan[%d] "
                "return_mode[%d] coordinate[%d] imu_rate[%d] extrinsic_src[%d]",
                cfg.broadcast_code, cfg.enable_connect, cfg.enable_fan,
                cfg.return_mode, cfg.coordinate, cfg.imu_rate,
                cfg.extrinsic_parameter_source);
    if (cfg.enable_connect) {
      if (!AddBroadcastCodeToWhitelist(cfg.broadcast_code)) {
        UserRawConfig config_copy = cfg;
        if (AddRawUserConfig(config_copy)) {
          RCLCPP_DEBUG(logger_, "Raw config already exists: %s", cfg.broadcast_code);
        }
      }
    }
  }

  if (whitelist_count_) {
    DisableAutoConnectMode();
    RCLCPP_INFO(logger_, "Disable auto connect mode");
    for (uint32_t i = 0; i < whitelist_count_; i++) {
      RCLCPP_INFO(logger_, "Whitelist[%u]: %s", i, broadcast_code_whitelist_[i]);
    }
  } else {
    EnableAutoConnectMode();
    RCLCPP_INFO(logger_, "No broadcast code in whitelist, using automatic connection mode");
  }

  enable_timesync_ = enable_timesync;
  timesync_config_ = ts_config;

  if (enable_timesync_) {
    timesync_ = TimeSync::GetInstance();
    if (timesync_->InitTimeSync(timesync_config_)) {
      RCLCPP_ERROR(logger_, "Timesync init fail");
      return -1;
    }
    if (timesync_->SetReceiveSyncTimeCb(ReceiveSyncTimeCallback, this)) {
      RCLCPP_ERROR(logger_, "Set Timesync callback fail");
      return -1;
    }
    timesync_->StartTimesync();
    RCLCPP_INFO(logger_, "Timesync enabled");
  } else {
    RCLCPP_INFO(logger_, "Timesync disabled");
  }

  /** Start livox sdk to receive lidar data */
  if (!Start()) {
    Uninit();
    RCLCPP_ERROR(logger_, "Livox-SDK start fail");
    return -1;
  }

  if (g_lds_ldiar == nullptr) {
    g_lds_ldiar = this;
  }
  is_initialized_ = true;
  RCLCPP_INFO(logger_, "Livox-SDK init success");

  return 0;
}

int LdsLidar::DeInitLdsLidar(void) {
  if (!is_initialized_) {
    RCLCPP_WARN(logger_, "LiDAR data source is not initialized");
    return -1;
  }

  Uninit();
  RCLCPP_INFO(logger_, "Livox SDK Deinit completely");

  if (timesync_) {
    timesync_->DeInitTimeSync();
  }

  is_initialized_ = false;
  whitelist_count_ = 0;
  memset(broadcast_code_whitelist_, 0, sizeof(broadcast_code_whitelist_));
  raw_config_.clear();
  g_lds_ldiar = nullptr;

  return 0;
}

void LdsLidar::PrepareExit(void) { DeInitLdsLidar(); }

/** Static function in LdsLidar for callback or event process ----------------*/

void LdsLidar::OnLidarDataCb(uint8_t handle, LivoxEthPacket *data,
                             uint32_t data_num, void *client_data) {
  LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data);
  LivoxEthPacket *eth_packet = data;

  if (!data || !data_num || (handle >= kMaxLidarCount)) {
    return;
  }

  lds_lidar->StorageRawPacket(handle, eth_packet);
}

void LdsLidar::OnDeviceBroadcast(const BroadcastDeviceInfo *info) {
  if (info == nullptr) {
    return;
  }

  if (info->dev_type == kDeviceTypeHub) {
    RCLCPP_WARN(g_lds_ldiar->logger_,
                "In lidar mode, can't connect a hub: %s", info->broadcast_code);
    return;
  }

  if (g_lds_ldiar->IsAutoConnectMode()) {
    RCLCPP_INFO(g_lds_ldiar->logger_,
                "Auto connection mode, will connect %s", info->broadcast_code);
  } else {
    if (!g_lds_ldiar->IsBroadcastCodeExistInWhitelist(info->broadcast_code)) {
      RCLCPP_DEBUG(g_lds_ldiar->logger_,
                   "Not in whitelist, skipping %s", info->broadcast_code);
      return;
    }
  }

  bool result = false;
  uint8_t handle = 0;
  result = AddLidarToConnect(info->broadcast_code, &handle);
  if (result == kStatusSuccess && handle < kMaxLidarCount) {
    SetDataCallback(handle, OnLidarDataCb, (void *)g_lds_ldiar);

    LidarDevice *p_lidar = &(g_lds_ldiar->lidars_[handle]);
    p_lidar->handle = handle;
    p_lidar->connect_state = kConnectStateOff;

    UserRawConfig config;
    if (g_lds_ldiar->GetRawConfig(info->broadcast_code, config)) {
      RCLCPP_DEBUG(g_lds_ldiar->logger_,
                   "No raw config found for %s, using defaults", info->broadcast_code);
      config.enable_fan = 1;
      config.return_mode = kFirstReturn;
      config.coordinate = kCoordinateCartesian;
      config.imu_rate = kImuFreq200Hz;
      config.extrinsic_parameter_source = kNoneExtrinsicParameter;
      config.enable_high_sensitivity = false;
    }

    p_lidar->config.enable_fan = config.enable_fan;
    p_lidar->config.return_mode = config.return_mode;
    p_lidar->config.coordinate = config.coordinate;
    p_lidar->config.imu_rate = config.imu_rate;
    p_lidar->config.extrinsic_parameter_source =
        config.extrinsic_parameter_source;
    p_lidar->config.enable_high_sensitivity = config.enable_high_sensitivity;
  } else {
    RCLCPP_ERROR(g_lds_ldiar->logger_,
                 "Add lidar to connect failed: %d %d", result, handle);
  }
}

void LdsLidar::OnDeviceChange(const DeviceInfo *info, DeviceEvent type) {
  if (info == nullptr) {
    return;
  }

  uint8_t handle = info->handle;
  if (handle >= kMaxLidarCount) {
    return;
  }

  LidarDevice *p_lidar = &(g_lds_ldiar->lidars_[handle]);
  if (type == kEventConnect) {
    QueryDeviceInformation(handle, DeviceInformationCb, g_lds_ldiar);
    if (p_lidar->connect_state == kConnectStateOff) {
      p_lidar->connect_state = kConnectStateOn;
      p_lidar->info = *info;
    }
  } else if (type == kEventDisconnect) {
    RCLCPP_WARN(g_lds_ldiar->logger_,
                "Lidar[%s] disconnect", info->broadcast_code);
    ResetLidar(p_lidar, kSourceRawLidar);
  } else if (type == kEventStateChange) {
    p_lidar->info = *info;
  }

  if (p_lidar->connect_state == kConnectStateOn) {
    RCLCPP_INFO(g_lds_ldiar->logger_,
                "Lidar[%s] status_code[%d] working_state[%d] feature[%d]",
                p_lidar->info.broadcast_code,
                p_lidar->info.status.status_code.error_code,
                p_lidar->info.state, p_lidar->info.feature);
    SetErrorMessageCallback(handle, LidarErrorStatusCb);

    if (p_lidar->info.state == kLidarStateNormal) {
      lock_guard<mutex> lock(g_lds_ldiar->config_mutex_);

      if (p_lidar->config.coordinate != 0) {
        SetSphericalCoordinate(handle, SetCoordinateCb, g_lds_ldiar);
      } else {
        SetCartesianCoordinate(handle, SetCoordinateCb, g_lds_ldiar);
      }
      p_lidar->config.set_bits |= kConfigCoordinate;

      if (kDeviceTypeLidarMid40 != info->type) {
        LidarSetPointCloudReturnMode(
            handle, (PointCloudReturnMode)(p_lidar->config.return_mode),
            SetPointCloudReturnModeCb, g_lds_ldiar);
        p_lidar->config.set_bits |= kConfigReturnMode;
      }

      if ((kDeviceTypeLidarMid70 != info->type) &&
          (kDeviceTypeLidarMid40 != info->type)) {
        LidarSetImuPushFrequency(handle, (ImuFreq)(p_lidar->config.imu_rate),
                                 SetImuRatePushFrequencyCb, g_lds_ldiar);
        p_lidar->config.set_bits |= kConfigImuRate;
      }

      if (p_lidar->config.extrinsic_parameter_source ==
          kExtrinsicParameterFromLidar) {
        LidarGetExtrinsicParameter(handle, GetLidarExtrinsicParameterCb,
                                   g_lds_ldiar);
        p_lidar->config.set_bits |= kConfigGetExtrinsicParameter;
      }

      if (kDeviceTypeLidarTele == info->type) {
        if (p_lidar->config.enable_high_sensitivity) {
          LidarEnableHighSensitivity(handle, SetHighSensitivityCb, g_lds_ldiar);
          RCLCPP_INFO(g_lds_ldiar->logger_, "Enable high sensitivity");
        } else {
          LidarDisableHighSensitivity(handle, SetHighSensitivityCb,
                                      g_lds_ldiar);
          RCLCPP_INFO(g_lds_ldiar->logger_, "Disable high sensitivity");
        }
        p_lidar->config.set_bits |= kConfigSetHighSensitivity;
      }

      p_lidar->connect_state = kConnectStateConfig;
    }
  }
}

void LdsLidar::DeviceInformationCb(livox_status status, uint8_t handle,
                                   DeviceInformationResponse *ack,
                                   void *clent_data) {
  if (status != kStatusSuccess) {
    RCLCPP_WARN(g_lds_ldiar->logger_, "Device Query Information Failed: %d", status);
  }
  if (ack) {
    RCLCPP_INFO(g_lds_ldiar->logger_, "Firmware version: %d.%d.%d.%d",
                ack->firmware_version[0], ack->firmware_version[1],
                ack->firmware_version[2], ack->firmware_version[3]);
  }
}

void LdsLidar::LidarErrorStatusCb(livox_status status, uint8_t handle,
                                  ErrorMessage *message) {
  static uint32_t error_message_count = 0;
  if (message != NULL) {
    ++error_message_count;
    if (0 == (error_message_count % 100)) {
      RCLCPP_WARN(g_lds_ldiar->logger_,
                  "Handle[%u] temp[%u] volt[%u] motor[%u] dirty[%u] "
                  "firmware[%u] pps[%u] fan[%u] heating[%u] ptp[%u] "
                  "timesync[%u] system[%u]",
                  handle,
                  message->lidar_error_code.temp_status,
                  message->lidar_error_code.volt_status,
                  message->lidar_error_code.motor_status,
                  message->lidar_error_code.dirty_warn,
                  message->lidar_error_code.firmware_err,
                  message->lidar_error_code.device_status,
                  message->lidar_error_code.fan_status,
                  message->lidar_error_code.self_heating,
                  message->lidar_error_code.ptp_status,
                  message->lidar_error_code.time_sync_status,
                  message->lidar_error_code.system_status);
    }
  }
}

void LdsLidar::ControlFanCb(livox_status status, uint8_t handle,
                            uint8_t response, void *clent_data) {}

void LdsLidar::SetPointCloudReturnModeCb(livox_status status, uint8_t handle,
                                         uint8_t response, void *clent_data) {
  LdsLidar *lds_lidar = static_cast<LdsLidar *>(clent_data);
  if (handle >= kMaxLidarCount) {
    return;
  }
  LidarDevice *p_lidar = &(lds_lidar->lidars_[handle]);

  if (status == kStatusSuccess) {
    RCLCPP_INFO(g_lds_ldiar->logger_, "Set return mode success");
    lock_guard<mutex> lock(lds_lidar->config_mutex_);
    p_lidar->config.set_bits &= ~((uint32_t)(kConfigReturnMode));
    if (!p_lidar->config.set_bits) {
      LidarStartSampling(handle, StartSampleCb, lds_lidar);
      p_lidar->connect_state = kConnectStateSampling;
    }
  } else {
    LidarSetPointCloudReturnMode(
        handle, (PointCloudReturnMode)(p_lidar->config.return_mode),
        SetPointCloudReturnModeCb, lds_lidar);
    RCLCPP_WARN(g_lds_ldiar->logger_, "Set return mode fail, retrying");
  }
}

void LdsLidar::SetCoordinateCb(livox_status status, uint8_t handle,
                               uint8_t response, void *clent_data) {
  LdsLidar *lds_lidar = static_cast<LdsLidar *>(clent_data);
  if (handle >= kMaxLidarCount) {
    return;
  }
  LidarDevice *p_lidar = &(lds_lidar->lidars_[handle]);

  if (status == kStatusSuccess) {
    RCLCPP_INFO(g_lds_ldiar->logger_, "Set coordinate success");
    lock_guard<mutex> lock(lds_lidar->config_mutex_);
    p_lidar->config.set_bits &= ~((uint32_t)(kConfigCoordinate));
    if (!p_lidar->config.set_bits) {
      LidarStartSampling(handle, StartSampleCb, lds_lidar);
      p_lidar->connect_state = kConnectStateSampling;
    }
  } else {
    if (p_lidar->config.coordinate != 0) {
      SetSphericalCoordinate(handle, SetCoordinateCb, lds_lidar);
    } else {
      SetCartesianCoordinate(handle, SetCoordinateCb, lds_lidar);
    }
    RCLCPP_WARN(g_lds_ldiar->logger_, "Set coordinate fail, retrying");
  }
}

void LdsLidar::SetImuRatePushFrequencyCb(livox_status status, uint8_t handle,
                                         uint8_t response, void *clent_data) {
  LdsLidar *lds_lidar = static_cast<LdsLidar *>(clent_data);
  if (handle >= kMaxLidarCount) {
    return;
  }
  LidarDevice *p_lidar = &(lds_lidar->lidars_[handle]);

  if (status == kStatusSuccess) {
    RCLCPP_INFO(g_lds_ldiar->logger_, "Set IMU rate success");
    lock_guard<mutex> lock(lds_lidar->config_mutex_);
    p_lidar->config.set_bits &= ~((uint32_t)(kConfigImuRate));
    if (!p_lidar->config.set_bits) {
      LidarStartSampling(handle, StartSampleCb, lds_lidar);
      p_lidar->connect_state = kConnectStateSampling;
    }
  } else {
    LidarSetImuPushFrequency(handle, (ImuFreq)(p_lidar->config.imu_rate),
                             SetImuRatePushFrequencyCb, g_lds_ldiar);
    RCLCPP_WARN(g_lds_ldiar->logger_, "Set IMU rate fail, retrying");
  }
}

void LdsLidar::GetLidarExtrinsicParameterCb(
    livox_status status, uint8_t handle,
    LidarGetExtrinsicParameterResponse *response, void *clent_data) {
  LdsLidar *lds_lidar = static_cast<LdsLidar *>(clent_data);
  if (handle >= kMaxLidarCount) {
    return;
  }

  if (status == kStatusSuccess) {
    if (response != nullptr) {
      RCLCPP_INFO(g_lds_ldiar->logger_,
                  "Lidar[%d] get ExtrinsicParameter status[%d] response[%d]",
                  handle, status, response->ret_code);
      LidarDevice *p_lidar = &(lds_lidar->lidars_[handle]);
      ExtrinsicParameter *p_extrinsic = &p_lidar->extrinsic_parameter;
      p_extrinsic->euler[0] = static_cast<float>(response->roll * PI / 180.0);
      p_extrinsic->euler[1] = static_cast<float>(response->pitch * PI / 180.0);
      p_extrinsic->euler[2] = static_cast<float>(response->yaw * PI / 180.0);
      p_extrinsic->trans[0] = static_cast<float>(response->x / 1000.0);
      p_extrinsic->trans[1] = static_cast<float>(response->y / 1000.0);
      p_extrinsic->trans[2] = static_cast<float>(response->z / 1000.0);
      EulerAnglesToRotationMatrix(p_extrinsic->euler, p_extrinsic->rotation);
      if (p_lidar->config.extrinsic_parameter_source) {
        p_extrinsic->enable = true;
      }
      RCLCPP_INFO(g_lds_ldiar->logger_,
                  "Lidar[%d] get ExtrinsicParameter success", handle);

      lock_guard<mutex> lock(lds_lidar->config_mutex_);
      p_lidar->config.set_bits &= ~((uint32_t)(kConfigGetExtrinsicParameter));
      if (!p_lidar->config.set_bits) {
        LidarStartSampling(handle, StartSampleCb, lds_lidar);
        p_lidar->connect_state = kConnectStateSampling;
      }
    } else {
      RCLCPP_WARN(g_lds_ldiar->logger_,
                  "Lidar[%d] get ExtrinsicParameter fail", handle);
    }
  } else if (status == kStatusTimeout) {
    RCLCPP_WARN(g_lds_ldiar->logger_,
                "Lidar[%d] get ExtrinsicParameter timeout", handle);
  }
}

void LdsLidar::SetHighSensitivityCb(livox_status status, uint8_t handle,
                                    DeviceParameterResponse *response,
                                    void *clent_data) {
  LdsLidar *lds_lidar = static_cast<LdsLidar *>(clent_data);
  if (handle >= kMaxLidarCount) {
    return;
  }
  LidarDevice *p_lidar = &(lds_lidar->lidars_[handle]);

  if (status == kStatusSuccess) {
    p_lidar->config.set_bits &= ~((uint32_t)(kConfigSetHighSensitivity));
    RCLCPP_INFO(g_lds_ldiar->logger_, "Set high sensitivity success");
    lock_guard<mutex> lock(lds_lidar->config_mutex_);
    if (!p_lidar->config.set_bits) {
      LidarStartSampling(handle, StartSampleCb, lds_lidar);
      p_lidar->connect_state = kConnectStateSampling;
    };
  } else {
    if (p_lidar->config.enable_high_sensitivity) {
      LidarEnableHighSensitivity(handle, SetHighSensitivityCb, g_lds_ldiar);
    } else {
      LidarDisableHighSensitivity(handle, SetHighSensitivityCb, g_lds_ldiar);
    }
    RCLCPP_WARN(g_lds_ldiar->logger_, "Set high sensitivity fail, retrying");
  }
}

void LdsLidar::StartSampleCb(livox_status status, uint8_t handle,
                             uint8_t response, void *clent_data) {
  LdsLidar *lds_lidar = static_cast<LdsLidar *>(clent_data);
  if (handle >= kMaxLidarCount) {
    return;
  }

  LidarDevice *p_lidar = &(lds_lidar->lidars_[handle]);
  if (status == kStatusSuccess) {
    if (response != 0) {
      p_lidar->connect_state = kConnectStateOn;
      RCLCPP_WARN(g_lds_ldiar->logger_,
                  "Lidar start sample fail: state[%d] handle[%d] res[%d]",
                  status, handle, response);
    } else {
      RCLCPP_INFO(g_lds_ldiar->logger_, "Lidar start sample success");
    }
  } else if (status == kStatusTimeout) {
    p_lidar->connect_state = kConnectStateOn;
    RCLCPP_WARN(g_lds_ldiar->logger_,
                "Lidar start sample timeout: state[%d] handle[%d] res[%d]",
                status, handle, response);
  }
}

void LdsLidar::StopSampleCb(livox_status status, uint8_t handle,
                            uint8_t response, void *clent_data) {}

void LdsLidar::SetRmcSyncTimeCb(livox_status status, uint8_t handle,
                                uint8_t response, void *client_data) {
  if (handle >= kMaxLidarCount) {
    return;
  }
  RCLCPP_DEBUG(g_lds_ldiar->logger_,
               "Set lidar[%d] sync time status[%d] response[%d]",
               handle, status, response);
}

void LdsLidar::ReceiveSyncTimeCallback(const char *rmc, uint32_t rmc_length,
                                       void *client_data) {
  LdsLidar *lds_lidar = static_cast<LdsLidar *>(client_data);
  LidarDevice *p_lidar = nullptr;
  for (uint8_t handle = 0; handle < kMaxLidarCount; handle++) {
    p_lidar = &(lds_lidar->lidars_[handle]);
    if (p_lidar->connect_state == kConnectStateSampling &&
        p_lidar->info.state == kLidarStateNormal) {
      livox_status status = LidarSetRmcSyncTime(handle, rmc, rmc_length,
                                                SetRmcSyncTimeCb, lds_lidar);
      if (status != kStatusSuccess) {
        RCLCPP_WARN(g_lds_ldiar->logger_,
                    "Set GPRMC sync time error code: %d", status);
      }
    }
  }
}

int LdsLidar::AddBroadcastCodeToWhitelist(const char *broadcast_code) {
  if (!broadcast_code || (strlen(broadcast_code) > kBroadcastCodeSize) ||
      (whitelist_count_ >= kMaxLidarCount)) {
    return -1;
  }

  if (LdsLidar::IsBroadcastCodeExistInWhitelist(broadcast_code)) {
    RCLCPP_DEBUG(logger_, "%s already in whitelist", broadcast_code);
    return -1;
  }

  strcpy(broadcast_code_whitelist_[whitelist_count_], broadcast_code);
  ++whitelist_count_;

  return 0;
}

bool LdsLidar::IsBroadcastCodeExistInWhitelist(const char *broadcast_code) {
  if (!broadcast_code) {
    return false;
  }

  for (uint32_t i = 0; i < whitelist_count_; i++) {
    if (strncmp(broadcast_code, broadcast_code_whitelist_[i],
                kBroadcastCodeSize) == 0) {
      return true;
    }
  }

  return false;
}

int LdsLidar::AddRawUserConfig(UserRawConfig &config) {
  if (IsExistInRawConfig(config.broadcast_code)) {
    return -1;
  }

  raw_config_.push_back(config);
  RCLCPP_DEBUG(logger_, "Add raw user config: %s", config.broadcast_code);

  return 0;
}

bool LdsLidar::IsExistInRawConfig(const char *broadcast_code) {
  if (broadcast_code == nullptr) {
    return false;
  }

  for (auto ite_config : raw_config_) {
    if (strncmp(ite_config.broadcast_code, broadcast_code,
                kBroadcastCodeSize) == 0) {
      return true;
    }
  }

  return false;
}

int LdsLidar::GetRawConfig(const char *broadcast_code, UserRawConfig &config) {
  if (broadcast_code == nullptr) {
    return -1;
  }

  for (auto ite_config : raw_config_) {
    if (strncmp(ite_config.broadcast_code, broadcast_code,
                kBroadcastCodeSize) == 0) {
      config.enable_fan = ite_config.enable_fan;
      config.return_mode = ite_config.return_mode;
      config.coordinate = ite_config.coordinate;
      config.imu_rate = ite_config.imu_rate;
      config.extrinsic_parameter_source = ite_config.extrinsic_parameter_source;
      config.enable_high_sensitivity = ite_config.enable_high_sensitivity;
      return 0;
    }
  }

  return -1;
}

}  // namespace livox_ros
