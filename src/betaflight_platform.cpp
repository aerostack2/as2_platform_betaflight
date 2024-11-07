// Copyright 2023 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * @file pixhawk_platform.cpp
 *
 * MavlinkPlatform class implementation
 *
 * @author Miguel Fernández Cortizas
 *         Rafael Pérez Seguí
 */

#include <array>
#include <memory>
#include <string>
#include <iostream>

#include "as2_platform_betaflight/betaflight_platform.hpp"
#include "msp/msp_msg.hpp"

double convert_deg_s_to_rad_s(double deg_s)
{
  return deg_s / 180.0 * M_PI;
}


void notImplemented()
{
  throw std::runtime_error("NOT IMPLEMENTED");
}


namespace as2_platform_betaflight
{

void BetaflightPlatform::readParameters()
{
  this->declare_parameter<std::string>("device");
  this->declare_parameter<int>("baudrate");
  this->declare_parameter<bool>("external_odom");

  this->declare_parameter<float>("yaw_rate.min");
  this->declare_parameter<float>("yaw_rate.max");
  this->declare_parameter<float>("pitch_rate.min");
  this->declare_parameter<float>("pitch_rate.max");
  this->declare_parameter<float>("roll_rate.min");
  this->declare_parameter<float>("roll_rate.max");
  this->declare_parameter<float>("thrust.min");
  this->declare_parameter<float>("thrust.max");


  base_link_frame_id_ = as2::tf::generateTfName(this, "base_link");
  odom_frame_id_ = as2::tf::generateTfName(this, "odom");

  device_ = this->get_parameter("device").as_string();
  baudrate_ = this->get_parameter("baudrate").as_int();
  external_odom_ = this->get_parameter("external_odom").as_bool();

  max_thrust_ = this->get_parameter("thrust.max").as_double();
  min_thrust_ = this->get_parameter("thrust.min").as_double();
  max_pitch_rate_ = convert_deg_s_to_rad_s(this->get_parameter("pitch_rate.max").as_double());
  min_pitch_rate_ = convert_deg_s_to_rad_s(this->get_parameter("pitch_rate.min").as_double());
  max_roll_rate_ = convert_deg_s_to_rad_s(this->get_parameter("roll_rate.max").as_double());
  min_roll_rate_ = convert_deg_s_to_rad_s(this->get_parameter("roll_rate.min").as_double());
  max_yaw_rate_ = convert_deg_s_to_rad_s(this->get_parameter("yaw_rate.max").as_double());
  min_yaw_rate_ = convert_deg_s_to_rad_s(this->get_parameter("yaw_rate.min").as_double());


  RCLCPP_INFO(this->get_logger(), "Device: %s", device_.c_str());
  RCLCPP_INFO(this->get_logger(), "Baudrate: %d", baudrate_);
  RCLCPP_INFO(this->get_logger(), "External odometry mode: %s", external_odom_ ? "true" : "false");
  RCLCPP_INFO(
    this->get_logger(), "Simulation mode: %s",
    this->get_parameter("use_sim_time").as_bool() ? "true" : "false");
  RCLCPP_INFO(this->get_logger(), "Thrust bounds: [%f, %f]", min_thrust_, max_thrust_);
  RCLCPP_INFO(this->get_logger(), "Pitch rate bounds: [%f, %f]", min_pitch_rate_, max_pitch_rate_);
  RCLCPP_INFO(this->get_logger(), "Roll rate bounds: [%f, %f]", min_roll_rate_, max_roll_rate_);
  RCLCPP_INFO(this->get_logger(), "Yaw rate bounds: [%f, %f]", min_yaw_rate_, max_yaw_rate_);
  computeControlSlopes();
}


BetaflightPlatform::BetaflightPlatform(const rclcpp::NodeOptions & options)
: as2::AerialPlatform(options)
{
  readParameters();
  configureSensors();
  initChannels();

  auto out = fcu_.connect(device_, baudrate_, 0.0, true);
  if (!out) {
    RCLCPP_ERROR(this->get_logger(), "Could not connect to device %s", device_.c_str());
    throw std::runtime_error("Could not connect to device");
  }
  fcu_.setLoggingLevel(msp::client::LoggingLevel::INFO);
  fcu_.setControlSource(fcu::ControlSource::MSP);

  fcu_.subscribe(&BetaflightPlatform::onStatus, this, 1);
  fcu_.subscribe(&BetaflightPlatform::onImu, this, 0.01);
  fcu_.subscribe(&BetaflightPlatform::onBattery, this, 0.1);
  // fcu_.subscribe(&BetaflightPlatform::onAltitude, this, 0.1);
  fcu_.subscribe(&BetaflightPlatform::onMotor, this, 0.1);
  fcu_.subscribe(&BetaflightPlatform::onRc, this, 0.1);
  box_names_ = fcu_.getBoxNames();
}

void BetaflightPlatform::configureSensors()
{
  imu_sensor_ptr_ = std::make_unique<as2::sensors::Imu>("imu", this);
  battery_sensor_ptr_ = std::make_unique<as2::sensors::Battery>("battery", this);
  // gps_sensor_ptr_ = std::make_unique<as2::sensors::GPS>("gps", this);

  // odometry_raw_estimation_ptr_ =
  //   std::make_unique<as2::sensors::Sensor<nav_msgs::msg::Odometry>>("odom", this);
}

bool BetaflightPlatform::ownSetArmingState(bool state)
{
  // TODO(miferco97): check if this is correct
  int value = state ? 1 : 0;
  channel_values_[RC_CHANNELS::ARM] = 1000 + value * 1000;
  return true;
}

bool BetaflightPlatform::ownSetOffboardControl(bool offboard)
{
  // TODO(miferco97): check if this is correct
  return true;
}

bool BetaflightPlatform::ownSetPlatformControlMode(const as2_msgs::msg::ControlMode & msg)
{
  // ONLY SUPPORTS ACRO MODE
  if (msg.control_mode != as2_msgs::msg::ControlMode::ACRO) {
    RCLCPP_WARN(this->get_logger(), "CONTROL MODE %d NOT SUPPORTED", msg.control_mode);
    return false;
  }
  return true;
}

bool BetaflightPlatform::ownSendCommand()
{
  // ONLY ACRO MODE IS SUPPORTED

  double thrust = this->command_thrust_msg_.thrust;
  double roll = this->command_twist_msg_.twist.angular.x;
  double pitch = this->command_twist_msg_.twist.angular.y;
  double yaw = this->command_twist_msg_.twist.angular.z;

  // saturate thrust
  thrust = std::clamp(thrust, min_thrust_, max_thrust_);
  roll = std::clamp(roll, min_roll_rate_, max_roll_rate_);
  pitch = std::clamp(pitch, min_pitch_rate_, max_pitch_rate_);
  yaw = std::clamp(yaw, min_yaw_rate_, max_yaw_rate_);

  // convert to pulse width
  uint16_t roll_pulse = static_cast<uint16_t>(1500 + roll / roll_slope_);
  uint16_t pitch_pulse = static_cast<uint16_t>(1500 + pitch / pitch_slope_);
  uint16_t yaw_pulse = static_cast<uint16_t>(1500 + yaw / roll_slope_);

  // thrust we must use thrust map to convert to pulse width
  // TODO(miferco97): implement thrust map
  uint16_t throttle_pulse = static_cast<uint16_t>(1000 + thrust / max_thrust_ * 1000);

  // set the values

  channel_values_[RC_CHANNELS::ROLL] = roll_pulse;
  channel_values_[RC_CHANNELS::PITCH] = pitch_pulse;
  channel_values_[RC_CHANNELS::THROTTLE] = throttle_pulse;
  channel_values_[RC_CHANNELS::YAW] = yaw_pulse;

  // print the values for debugging
  // std::cout << "Channel values: ";
  // for (auto & value : channel_values_) {
  //   std::cout << value << " ";
  // }
  // std::cout << std::endl;

  bool out = fcu_.setRc(channel_values_);
  if (!out) {
    RCLCPP_ERROR(this->get_logger(), "Could not send command to flight controller");
    return false;
  }
  return true;
}

void BetaflightPlatform::ownKillSwitch()
{
  // set all channels to 0
  std::fill(channel_values_.begin(), channel_values_.end(), 1000);

  channel_values_[RC_CHANNELS::KILLSWITCH] = 2000;
  while (true) {
    fcu_.setRc(channel_values_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void BetaflightPlatform::ownStopPlatform() {RCLCPP_WARN(this->get_logger(), "NOT IMPLEMENTED");}

void BetaflightPlatform::onStatus(const msp::msg::Status & status)
{
  if (!status.isHealthy()) {
    RCLCPP_WARN(this->get_logger(), "Flight controller is not healthy");
  }
}

void BetaflightPlatform::onImu(const msp::msg::RawImu & imu)
{
  const msp::msg::ImuSI imu_si(imu, 512.0, 1.0 / 4.096, 0.092, 9.80665);

  std_msgs::msg::Header hdr;
  hdr.stamp = this->get_clock()->now();
  hdr.frame_id = base_link_frame_id_;

  // raw imu data without orientation
  sensor_msgs::msg::Imu imu_msg;
  imu_msg.header = hdr;
  imu_msg.linear_acceleration.x = imu_si.acc[0];
  imu_msg.linear_acceleration.y = imu_si.acc[1];
  imu_msg.linear_acceleration.z = imu_si.acc[2];
  imu_msg.angular_velocity.x = imu_si.gyro[0] / 180.0 * M_PI;
  imu_msg.angular_velocity.y = imu_si.gyro[1] / 180.0 * M_PI;
  imu_msg.angular_velocity.z = imu_si.gyro[2] / 180.0 * M_PI;

  // magnetic field vector
  // sensor_msgs::msg::MagneticField mag_msg;
  // mag_msg.header = hdr;
  // mag_msg.magnetic_field.x = imu_si.mag[0] * 1e-6;
  // mag_msg.magnetic_field.y = imu_si.mag[1] * 1e-6;
  // mag_msg.magnetic_field.z = imu_si.mag[2] * 1e-6;

  imu_sensor_ptr_->updateAndPublish(imu_msg);
}

// void BetaflightPlatform::onAltitude(const msp::msg::Altitude & altitude)
// {
//   std::cout << "Altitude: " << altitude << std::endl;
// }

void BetaflightPlatform::onMotor(const msp::msg::Motor & motor)
{
  // std::cout << "Motor: " << motor << std::endl;
}

void BetaflightPlatform::onBattery(const msp::msg::BatteryState & battery)
{
  sensor_msgs::msg::BatteryState battery_msg;
  battery_msg.header.stamp = this->get_clock()->now();
  battery_msg.voltage = battery.voltage;
  battery_msg.current = battery.amperage;
  battery_msg.percentage = battery.voltage / (battery.cell_count * 4.2);
  battery_msg.charge = battery.capacity_mAh;

  // std::cout << "Battery: " << battery << std::endl;
}


}  // namespace as2_platform_betaflight
