// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

// local
#include "romea_ublox/ublox_driver.hpp"

// ros
#include <rclcpp_components/register_node_macro.hpp>

// romea ros
#include <romea_common_utils/qos.hpp>

// std
#include <memory>
#include <string>

namespace romea
{

//-----------------------------------------------------------------------------
UbloxDriver::UbloxDriver(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp::Node>("ublox_driver", options)),
  gps_interface_(node_),
  gps_data_(node_),
  rtcm_sub_(nullptr),
  thread_(&UbloxDriver::thread_callback, this)
{
  auto callback = std::bind(&UbloxDriver::rtcm_callback_, this, std::placeholders::_1);

  rtcm_sub_ = node_->create_subscription<mavros_msgs::msg::RTCM>(
    "ntrip/rtcm", best_effort(1), callback);
}

//-----------------------------------------------------------------------------
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
UbloxDriver::get_node_base_interface() const
{
  return node_->get_node_base_interface();
}

//-----------------------------------------------------------------------------
void UbloxDriver::rtcm_callback_(mavros_msgs::msg::RTCM::SharedPtr msg)
{
  std::string data(msg->data.begin(), msg->data.end());
  gps_interface_.write(data);
}

//-----------------------------------------------------------------------------
void UbloxDriver::thread_callback()
{
  while (rclcpp::ok()) {
    auto nmea_sentence = gps_interface_.read_nmea_sentence();
    if (nmea_sentence.has_value()) {
      gps_data_.process_nmea_sentence(nmea_sentence.value());
    }
  }
}

}  // namespace romea

RCLCPP_COMPONENTS_REGISTER_NODE(romea::UbloxDriver)
