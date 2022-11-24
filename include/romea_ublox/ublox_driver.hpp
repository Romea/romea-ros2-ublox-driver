// romea
#include <romea_gps_utils/gps_serial_interface.hpp>
#include <romea_gps_utils/gps_data.hpp>
#include "romea_ublox/visibility_control.h"

// ros
#include <mavros_msgs/msg/rtcm.hpp>

namespace romea {


class UbloxDriver
{
public:
  ROMEA_UBLOX_PUBLIC
  explicit UbloxDriver(const rclcpp::NodeOptions & options);

  ROMEA_UBLOX_PUBLIC
  virtual ~UbloxDriver() = default;

  ROMEA_UBLOX_PUBLIC
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface() const;

private:
  void rtcm_callback_(mavros_msgs::msg::RTCM::SharedPtr msg);

  void thread_callback();

private:
   rclcpp::Node::SharedPtr node_;
   GpsSerialInterface gps_interface_;
   GpsData gps_data_;

   rclcpp::Subscription<mavros_msgs::msg::RTCM>::SharedPtr rtcm_sub_;
   std::thread thread_;
};

}  // namespace romea
