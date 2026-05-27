#ifndef MANIPULATOR_HARDWARE__MANIPULATOR_HARDWARE_INTERFACE_HPP_
#define MANIPULATOR_HARDWARE__MANIPULATOR_HARDWARE_INTERFACE_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace manipulator_hardware
{

class ManipulatorHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ManipulatorHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  std::vector<std::string> joint_names_;

  std::vector<double> hw_positions_;
  std::vector<double> hw_previous_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;

  std::vector<double> last_sent_commands_;

  std::string serial_port_;
  int baud_rate_;
  int timeout_ms_;
  std::string control_mode_;

  int serial_fd_{-1};
  uint32_t sequence_{0};
  std::string rx_buffer_;

  bool openSerialPort();
  void closeSerialPort();

  bool writeLine(const std::string & line);
  bool readLine(std::string & line);

  double radToDeg(double rad) const;
};

}  // namespace manipulator_hardware

#endif  // MANIPULATOR_HARDWARE__MANIPULATOR_HARDWARE_INTERFACE_HPP_