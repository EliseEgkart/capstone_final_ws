#include "manipulator_hardware/manipulator_hardware_interface.hpp"

#include <cmath>
#include <limits>
#include <sstream>
#include <iomanip>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <exception>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace manipulator_hardware
{

namespace
{

constexpr double kCommandEpsilonRad = 1e-3;
constexpr double kWritePeriodMs = 50.0;   // 10 Hz serial command rate
constexpr int kMaxAckRetries = 3;

bool g_waiting_ack = false;
uint32_t g_pending_seq = 0;
int g_retry_count = 0;
std::string g_pending_packet;
std::vector<double> g_pending_commands;
rclcpp::Time g_last_tx_time;
rclcpp::Time g_last_write_time;
bool g_last_tx_time_valid = false;
bool g_last_write_time_valid = false;

void resetSerialAckState()
{
  g_waiting_ack = false;
  g_pending_seq = 0;
  g_retry_count = 0;
  g_pending_packet.clear();
  g_pending_commands.clear();
  g_last_tx_time_valid = false;
  g_last_write_time_valid = false;
}

speed_t getBaudRate(int baud_rate)
{
  switch (baud_rate)
  {
    case 9600:
      return B9600;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    default:
      return B115200;
  }
}

}  // namespace

hardware_interface::CallbackReturn ManipulatorHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_.clear();

  const size_t joint_count = info_.joints.size();

  hw_positions_.assign(joint_count, 0.0);
  hw_previous_positions_.assign(joint_count, 0.0);
  hw_velocities_.assign(joint_count, 0.0);
  hw_commands_.assign(joint_count, 0.0);
  last_sent_commands_.assign(
    joint_count,
    std::numeric_limits<double>::quiet_NaN());

  for (size_t i = 0; i < joint_count; ++i)
  {
    const auto & joint = info_.joints[i];
    joint_names_.push_back(joint.name);

    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("ManipulatorHardwareInterface"),
        "Joint '%s' must have exactly one position command interface.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    bool has_position_state = false;
    bool has_velocity_state = false;

    for (const auto & state_interface : joint.state_interfaces)
    {
      if (state_interface.name == hardware_interface::HW_IF_POSITION)
      {
        has_position_state = true;

        if (!state_interface.initial_value.empty())
        {
          hw_positions_[i] = std::stod(state_interface.initial_value);
          hw_commands_[i] = hw_positions_[i];
        }
      }

      if (state_interface.name == hardware_interface::HW_IF_VELOCITY)
      {
        has_velocity_state = true;
      }
    }

    if (!has_position_state || !has_velocity_state)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("ManipulatorHardwareInterface"),
        "Joint '%s' must have position and velocity state interfaces.",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    hw_previous_positions_[i] = hw_positions_[i];
  }

  serial_port_ = info_.hardware_parameters.count("serial_port")
    ? info_.hardware_parameters.at("serial_port")
    : "/dev/ttyUSB_ESP32";

  baud_rate_ = info_.hardware_parameters.count("baud_rate")
    ? std::stoi(info_.hardware_parameters.at("baud_rate"))
    : 115200;

  timeout_ms_ = info_.hardware_parameters.count("timeout_ms")
    ? std::stoi(info_.hardware_parameters.at("timeout_ms"))
    : 50;

  control_mode_ = info_.hardware_parameters.count("control_mode")
    ? info_.hardware_parameters.at("control_mode")
    : "open_loop";

  RCLCPP_INFO(
    rclcpp::get_logger("ManipulatorHardwareInterface"),
    "Initialized hardware interface: port=%s, baud=%d, timeout=%d ms, mode=%s",
    serial_port_.c_str(),
    baud_rate_,
    timeout_ms_,
    control_mode_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ManipulatorHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    state_interfaces.emplace_back(
      joint_names_[i],
      hardware_interface::HW_IF_POSITION,
      &hw_positions_[i]);

    state_interfaces.emplace_back(
      joint_names_[i],
      hardware_interface::HW_IF_VELOCITY,
      &hw_velocities_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ManipulatorHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    command_interfaces.emplace_back(
      joint_names_[i],
      hardware_interface::HW_IF_POSITION,
      &hw_commands_[i]);
  }

  return command_interfaces;
}

bool ManipulatorHardwareInterface::openSerialPort()
{
  serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (serial_fd_ < 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("ManipulatorHardwareInterface"),
      "Failed to open serial port '%s': %s",
      serial_port_.c_str(),
      std::strerror(errno));
    return false;
  }

  struct termios tty;
  std::memset(&tty, 0, sizeof(tty));

  if (tcgetattr(serial_fd_, &tty) != 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("ManipulatorHardwareInterface"),
      "Failed to get serial attributes: %s",
      std::strerror(errno));
    closeSerialPort();
    return false;
  }

  cfmakeraw(&tty);

  const speed_t baud = getBaudRate(baud_rate_);
  cfsetispeed(&tty, baud);
  cfsetospeed(&tty, baud);

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("ManipulatorHardwareInterface"),
      "Failed to set serial attributes: %s",
      std::strerror(errno));
    closeSerialPort();
    return false;
  }

  tcflush(serial_fd_, TCIOFLUSH);

  RCLCPP_INFO(
    rclcpp::get_logger("ManipulatorHardwareInterface"),
    "Opened serial port: %s at %d baud",
    serial_port_.c_str(),
    baud_rate_);

  return true;
}

void ManipulatorHardwareInterface::closeSerialPort()
{
  if (serial_fd_ >= 0)
  {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool ManipulatorHardwareInterface::writeLine(const std::string & line)
{
  if (serial_fd_ < 0)
  {
    return false;
  }

  size_t total_written = 0;

  while (total_written < line.size())
  {
    const ssize_t written = ::write(
      serial_fd_,
      line.c_str() + total_written,
      line.size() - total_written);

    if (written > 0)
    {
      total_written += static_cast<size_t>(written);
      continue;
    }

    if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
    {
      usleep(1000);
      continue;
    }

    RCLCPP_WARN(
      rclcpp::get_logger("ManipulatorHardwareInterface"),
      "Serial write failed: %s",
      std::strerror(errno));
    return false;
  }

  return true;
}

bool ManipulatorHardwareInterface::readLine(std::string & line)
{
  if (serial_fd_ < 0)
  {
    return false;
  }

  char buffer[256];
  const ssize_t n = ::read(serial_fd_, buffer, sizeof(buffer));

  if (n > 0)
  {
    rx_buffer_.append(buffer, buffer + n);
  }
  else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
  {
    RCLCPP_WARN(
      rclcpp::get_logger("ManipulatorHardwareInterface"),
      "Serial read failed: %s",
      std::strerror(errno));
    return false;
  }

  const auto pos = rx_buffer_.find('\n');

  if (pos == std::string::npos)
  {
    return false;
  }

  line = rx_buffer_.substr(0, pos);
  rx_buffer_.erase(0, pos + 1);

  if (!line.empty() && line.back() == '\r')
  {
    line.pop_back();
  }

  return true;
}

double ManipulatorHardwareInterface::radToDeg(double rad) const
{
  return rad * 180.0 / 3.14159265358979323846;
}

hardware_interface::CallbackReturn ManipulatorHardwareInterface::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    rclcpp::get_logger("ManipulatorHardwareInterface"),
    "Configuring manipulator hardware interface.");

  if (!openSerialPort())
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("ManipulatorHardwareInterface"),
      "Failed to configure serial connection.");

    return hardware_interface::CallbackReturn::ERROR;
  }

  // ESP32 boards can reset when the USB serial port is opened.
  // Wait before the first PING so the MCU can finish Serial.begin() and setup().
  usleep(1500 * 1000);
  tcflush(serial_fd_, TCIOFLUSH);

  for (int i = 0; i < 3; ++i)
  {
    writeLine("PING\r\n");
    usleep(100 * 1000);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ManipulatorHardwareInterface::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    rclcpp::get_logger("ManipulatorHardwareInterface"),
    "Activating manipulator hardware interface.");

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    hw_commands_[i] = hw_positions_[i];
    hw_previous_positions_[i] = hw_positions_[i];
    hw_velocities_[i] = 0.0;
  }

  for (size_t i = 0; i < last_sent_commands_.size(); ++i)
  {
    last_sent_commands_[i] = std::numeric_limits<double>::quiet_NaN();
  }

  resetSerialAckState();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ManipulatorHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    rclcpp::get_logger("ManipulatorHardwareInterface"),
    "Deactivating manipulator hardware interface.");

  writeLine("STOP\r\n");
  closeSerialPort();
  resetSerialAckState();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ManipulatorHardwareInterface::read(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  std::string line;

  while (readLine(line))
  {
    if (line.rfind("ACK,", 0) == 0)
    {
      RCLCPP_INFO(
        rclcpp::get_logger("ManipulatorHardwareInterface"),
        "RX: %s",
        line.c_str());

      try
      {
        const uint32_t ack_seq =
          static_cast<uint32_t>(std::stoul(line.substr(4)));

        if (g_waiting_ack && ack_seq == g_pending_seq)
        {
          if (g_pending_commands.size() == last_sent_commands_.size())
          {
            last_sent_commands_ = g_pending_commands;
          }

          g_waiting_ack = false;
          g_retry_count = 0;
          g_pending_packet.clear();
          g_pending_commands.clear();
          g_last_tx_time_valid = false;

          RCLCPP_INFO(
            rclcpp::get_logger("ManipulatorHardwareInterface"),
            "ACK matched. seq=%u",
            ack_seq);
        }
        else
        {
          RCLCPP_WARN(
            rclcpp::get_logger("ManipulatorHardwareInterface"),
            "ACK seq mismatch or stale ACK. rx=%u, pending=%u, waiting=%s",
            ack_seq,
            g_pending_seq,
            g_waiting_ack ? "true" : "false");
        }
      }
      catch (const std::exception &)
      {
        RCLCPP_WARN(
          rclcpp::get_logger("ManipulatorHardwareInterface"),
          "Invalid ACK format: %s",
          line.c_str());
      }
    }
    else if (line.rfind("ACK_STOP", 0) == 0 ||
             line.rfind("PONG", 0) == 0 ||
             line.rfind("ERR", 0) == 0 ||
             line.rfind("BOOT", 0) == 0)
    {
      RCLCPP_INFO(
        rclcpp::get_logger("ManipulatorHardwareInterface"),
        "RX: %s",
        line.c_str());
    }
    else if (line.rfind("STATE", 0) == 0)
    {
      RCLCPP_DEBUG(
        rclcpp::get_logger("ManipulatorHardwareInterface"),
        "RX: %s",
        line.c_str());
    }
    else
    {
      RCLCPP_INFO(
        rclcpp::get_logger("ManipulatorHardwareInterface"),
        "RX: %s",
        line.c_str());
    }
  }

  const double dt = period.seconds();

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    hw_previous_positions_[i] = hw_positions_[i];

    if (control_mode_ == "open_loop")
    {
      hw_positions_[i] = hw_commands_[i];
    }
    else
    {
      // 다음 단계에서 STATE 메시지를 파싱해서 실제 피드백 값을 넣는다.
      hw_positions_[i] = hw_commands_[i];
    }

    if (dt > 0.0)
    {
      hw_velocities_[i] =
        (hw_positions_[i] - hw_previous_positions_[i]) / dt;
    }
    else
    {
      hw_velocities_[i] = 0.0;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ManipulatorHardwareInterface::write(
  const rclcpp::Time & time,
  const rclcpp::Duration &)
{
  if (hw_commands_.size() < 4)
  {
    return hardware_interface::return_type::ERROR;
  }

  if (g_waiting_ack)
  {
    const double elapsed_ms = g_last_tx_time_valid
      ? (time - g_last_tx_time).seconds() * 1000.0
      : timeout_ms_ + 1.0;

    if (elapsed_ms < timeout_ms_)
    {
      return hardware_interface::return_type::OK;
    }

    if (g_retry_count >= kMaxAckRetries)
    {
      RCLCPP_WARN(
        rclcpp::get_logger("ManipulatorHardwareInterface"),
        "ACK timeout. Give up seq=%u after %d retries.",
        g_pending_seq,
        g_retry_count);

      g_waiting_ack = false;
      g_retry_count = 0;
      g_pending_packet.clear();
      g_pending_commands.clear();
      g_last_tx_time_valid = false;

      return hardware_interface::return_type::ERROR;
    }

    RCLCPP_WARN(
      rclcpp::get_logger("ManipulatorHardwareInterface"),
      "ACK timeout. Retry seq=%u (%d/%d)",
      g_pending_seq,
      g_retry_count + 1,
      kMaxAckRetries);

    if (!writeLine(g_pending_packet))
    {
      return hardware_interface::return_type::ERROR;
    }

    g_last_tx_time = time;
    g_last_tx_time_valid = true;
    ++g_retry_count;

    return hardware_interface::return_type::OK;
  }

  if (g_last_write_time_valid)
  {
    const double elapsed_ms = (time - g_last_write_time).seconds() * 1000.0;

    if (elapsed_ms < kWritePeriodMs)
    {
      return hardware_interface::return_type::OK;
    }
  }

  bool changed = false;

  for (size_t i = 0; i < hw_commands_.size(); ++i)
  {
    if (std::isnan(last_sent_commands_[i]) ||
        std::fabs(hw_commands_[i] - last_sent_commands_[i]) > kCommandEpsilonRad)
    {
      changed = true;
      break;
    }
  }

  if (!changed)
  {
    return hardware_interface::return_type::OK;
  }

  const double j1_deg = radToDeg(hw_commands_[0]);
  const double j2_deg = radToDeg(hw_commands_[1]);
  const double j3_deg = radToDeg(hw_commands_[2]);
  const double j4_deg = radToDeg(hw_commands_[3]);

  const uint32_t seq = sequence_++;

  std::ostringstream ss;
  ss << std::fixed << std::setprecision(3);
  ss << "CMD_POS,"
    << seq << ","
    << j1_deg << ","
    << j2_deg << ","
    << j3_deg << ","
    << j4_deg << ","
    << 1 << "\r\n";

  const std::string packet = ss.str();

  RCLCPP_INFO(
    rclcpp::get_logger("ManipulatorHardwareInterface"),
    "TX: %s",
    packet.c_str());

  if (!writeLine(packet))
  {
    return hardware_interface::return_type::ERROR;
  }

  g_pending_seq = seq;
  g_pending_packet = packet;
  g_pending_commands = hw_commands_;
  g_waiting_ack = true;
  g_retry_count = 0;
  g_last_tx_time = time;
  g_last_tx_time_valid = true;
  g_last_write_time = time;
  g_last_write_time_valid = true;

  // Do not update last_sent_commands_ here.
  // It is updated only after RX: ACK,<seq> is received in read().

  return hardware_interface::return_type::OK;
}

}  // namespace manipulator_hardware

PLUGINLIB_EXPORT_CLASS(
  manipulator_hardware::ManipulatorHardwareInterface,
  hardware_interface::SystemInterface)