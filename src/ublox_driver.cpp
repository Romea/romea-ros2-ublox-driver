#include "romea_ublox/ublox_driver.hpp"

namespace romea{

//-----------------------------------------------------------------------------
UbloxDriver::UbloxDriver(const rclcpp::NodeOptions & options):
  node_(std::make_shared<rclcpp::Node>("ublox_driver",options)),
  gps_interface_(node_),
  gps_data_(node_),
  thread_(&UbloxDriver::thread_callback,this)
{
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
  while (rclcpp::ok())
  {
    auto nmea_sentence= gps_interface_.read_nmea_sentence();
    if( nmea_sentence.has_value())
    {
      gps_data_.process_nmea_sentence(nmea_sentence.value());
    }
  }
}

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(romea::UbloxDriver)

