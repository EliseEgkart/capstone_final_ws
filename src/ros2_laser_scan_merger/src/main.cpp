#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <string>
#include <vector>
#include <array>
#include <iostream>

class scanMerger : public rclcpp::Node
{
public:
  scanMerger() : Node("ros2_laser_scan_merger")
  {
    initialize_params();
    refresh_params();

    laser1_ = std::make_shared<sensor_msgs::msg::LaserScan>();
    laser2_ = std::make_shared<sensor_msgs::msg::LaserScan>();

    auto default_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    sub1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        topic1_, default_qos, std::bind(&scanMerger::scan_callback1, this, std::placeholders::_1));
    sub2_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        topic2_, default_qos, std::bind(&scanMerger::scan_callback2, this, std::placeholders::_1));

    lift_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/lift", default_qos, std::bind(&scanMerger::lift_callback, this, std::placeholders::_1));

    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloudTopic_, rclcpp::SensorDataQoS());
    RCLCPP_INFO(this->get_logger(), "Hello");
  }

private:
  static constexpr float THRESHOLD_1 = -132.5;
  static constexpr float THRESHOLD_2 = -47.5;
  static constexpr float THRESHOLD_3 = 47.5;
  static constexpr float THRESHOLD_4 = 132.5;
  static constexpr float THRESHOLD_0 = 35.0;
  static constexpr float THRESHOLD_DIST = 0.70/2.0 + 0.40;
  static constexpr float THRESHOLD_x = (0.400+0.050)/2.0;
  static constexpr float THRESHOLD_y = (0.600+0.050)/2.0;
  

  void scan_callback1(const sensor_msgs::msg::LaserScan::SharedPtr _msg)
  {
    laser1_ = _msg;
    update_point_cloud_rgb();
    // RCLCPP_INFO(this->get_logger(), "I heard: '%f' '%f'", _msg->ranges[0],
    //         _msg->ranges[100]);
  }
  
  void scan_callback2(const sensor_msgs::msg::LaserScan::SharedPtr _msg)
  {
    laser2_ = _msg;
    // RCLCPP_INFO(this->get_logger(), "I heard: '%f' '%f'", _msg->ranges[0],
    //         _msg->ranges[100]);
  }

  void lift_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    is_lifted_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Lift state: '%s'", is_lifted_ ? "True" : "False");
  }

  void update_point_cloud_rgb()
  {
    refresh_params();
    pcl::PointCloud<pcl::PointXYZRGB> cloud_;
    std::vector<pcl::PointXYZRGB> points;
    std::vector<std::array<float, 2>> scan_data;
    int count = 0;
    float min_theta = 0;
    float max_theta = 0;
    if (show1_ && laser1_)
    {
      float temp_min_, temp_max_;
      if( laser1_->angle_min < laser1_->angle_max){
        temp_min_ = laser1_->angle_min;
        temp_max_ = laser1_->angle_max;
      } else{
        temp_min_ = laser1_->angle_max;
        temp_max_ = laser1_->angle_min;
      }
      for (float i = temp_min_; i <= temp_max_ && count < laser1_->ranges.size();
           i += laser1_->angle_increment)
      {
        pcl::PointXYZRGB pt;
        pt = pcl::PointXYZRGB(laser1R_, laser1G_, laser1B_);
        int used_count_ = count;
        if (flip1_)
        {
          used_count_ = (int)laser1_->ranges.size() - 1 - count;
        }
        float temp_x = laser1_->ranges[used_count_] * std::cos(i);
        float temp_y = laser1_->ranges[used_count_] * std::sin(i);
        pt.x =
            temp_x * std::cos(laser1Alpha_ * M_PI / 180) - temp_y * std::sin(laser1Alpha_ * M_PI / 180) + laser1XOff_;
        pt.y =
            temp_x * std::sin(laser1Alpha_ * M_PI / 180) + temp_y * std::cos(laser1Alpha_ * M_PI / 180) + laser1YOff_;
        pt.z = laser1ZOff_;
        if ((i < (laser1AngleMin_ * M_PI / 180)) || (i > (laser1AngleMax_ * M_PI / 180)))
        {
          if (inverse1_)
          {
            points.push_back(pt);
            float r_ = GET_R(pt.x, pt.y);
            float theta_ = GET_THETA(pt.x, pt.y);
            std::array<float, 2> res_;
            res_[1] = r_;
            res_[0] = theta_;
            scan_data.push_back(res_);
            if (theta_ < min_theta)
            {
              min_theta = theta_;
            }
            if (theta_ > max_theta)
            {
              max_theta = theta_;
            }
          }
        }
        else
        {
          if (!inverse1_)
          {
            points.push_back(pt);
            float r_ = GET_R(pt.x, pt.y);
            float theta_ = GET_THETA(pt.x, pt.y);
            std::array<float, 2> res_;
            res_[1] = r_;
            res_[0] = theta_;
            scan_data.push_back(res_);
            if (theta_ < min_theta)
            {
              min_theta = theta_;
            }
            if (theta_ > max_theta)
            {
              max_theta = theta_;
            }
          }
        }
        count++;
      }
    }

    count = 0;
    if (show2_ && laser2_)
    {
      float temp_min_, temp_max_;
      if( laser2_->angle_min < laser2_->angle_max){
        temp_min_ = laser2_->angle_min;
        temp_max_ = laser2_->angle_max;
      } else{
        temp_min_ = laser2_->angle_max;
        temp_max_ = laser2_->angle_min;
      }
      for (float i = temp_min_; i <= temp_max_ && count < laser2_->ranges.size();
           i += laser2_->angle_increment)
      {
        pcl::PointXYZRGB pt;
        pt = pcl::PointXYZRGB(laser2R_, laser2G_, laser2B_);

        int used_count_ = count;
        if (flip2_)
        {
          used_count_ = (int)laser2_->ranges.size() - 1 - count;
        }

        float temp_x = laser2_->ranges[used_count_] * std::cos(i);
        float temp_y = laser2_->ranges[used_count_] * std::sin(i);
        pt.x =
            temp_x * std::cos(laser2Alpha_ * M_PI / 180) - temp_y * std::sin(laser2Alpha_ * M_PI / 180) + laser2XOff_;
        pt.y =
            temp_x * std::sin(laser2Alpha_ * M_PI / 180) + temp_y * std::cos(laser2Alpha_ * M_PI / 180) + laser2YOff_;
        pt.z = laser2ZOff_;
        if ((i < (laser2AngleMin_ * M_PI / 180)) || (i > (laser2AngleMax_ * M_PI / 180)))
        {
          if (inverse2_)
          {
            points.push_back(pt);
            float r_ = GET_R(pt.x, pt.y);
            float theta_ = GET_THETA(pt.x, pt.y);
            std::array<float, 2> res_;
            res_[1] = r_;
            res_[0] = theta_;
            scan_data.push_back(res_);
            if (theta_ < min_theta)
            {
              min_theta = theta_;
            }
            if (theta_ > max_theta)
            {
              max_theta = theta_;
            }
          }
        }
        else
        {
          if (!inverse2_)
          {
            points.push_back(pt);
            float r_ = GET_R(pt.x, pt.y);
            float theta_ = GET_THETA(pt.x, pt.y);
            std::array<float, 2> res_;
            res_[1] = r_;
            res_[0] = theta_;
            scan_data.push_back(res_);
            if (theta_ < min_theta)
            {
              min_theta = theta_;
            }
            if (theta_ > max_theta)
            {
              max_theta = theta_;
            }
          }
        }
        count++;
      }
    }

    std::map<std::pair<int, int>, pcl::PointXYZRGB> point_map;
    for (const auto& pt : points) {
      float angle = std::atan2(pt.y, pt.x) * 180 / M_PI; // 각도를 라디안에서 도 단위로 변환
      float distance = GET_R(pt.x, pt.y); // 포인트의 거리 계산

      if (!is_lifted_ ||  (distance > THRESHOLD_DIST) ||
                          (angle < THRESHOLD_1 - THRESHOLD_0 || angle > THRESHOLD_1 + THRESHOLD_0) && 
                          (angle < THRESHOLD_2 - THRESHOLD_0 || angle > THRESHOLD_2 + THRESHOLD_0) && 
                          (angle < THRESHOLD_3 - THRESHOLD_0 || angle > THRESHOLD_3 + THRESHOLD_0) && 
                          (angle < THRESHOLD_4 - THRESHOLD_0 || angle > THRESHOLD_4 + THRESHOLD_0)) // 각도 필터링
      {
        // if (abs(pt.x) > THRESHOLD_x and abs(pt.y) > THRESHOLD_y)
        // {
          int x_key = static_cast<int>(pt.x * 100); // 필요에 따라 해상도를 조정
          int y_key = static_cast<int>(pt.y * 100); // 필요에 따라 해상도를 조정
          auto key = std::make_pair(x_key, y_key);
          if (point_map.find(key) == point_map.end())
          {
              point_map[key] = pt;
          }
          else
          {
              auto& existing_pt = point_map[key];
              existing_pt.x = (existing_pt.x + pt.x) / 2;
              existing_pt.y = (existing_pt.y + pt.y) / 2;
              existing_pt.z = (existing_pt.z + pt.z) / 2;
              existing_pt.r = (existing_pt.r + pt.r) / 2;
              existing_pt.g = (existing_pt.g + pt.g) / 2;
              existing_pt.b = (existing_pt.b + pt.b) / 2;
          }
        }
      // }
    }

    for (const auto& item : point_map) {
        cloud_.points.push_back(item.second);
    }

    // 포인트 클라우드를 ROS 메시지로 변환
    auto pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(cloud_, *pc2_msg_);
    pc2_msg_->header.frame_id = cloudFrameId_;
    pc2_msg_->header.stamp = now();
    pc2_msg_->is_dense = false;
    point_cloud_pub_->publish(*pc2_msg_);

    // auto pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();
    // pcl::toROSMsg(cloud_, *pc2_msg_);
    // pc2_msg_->header.frame_id = cloudFrameId_;
    // pc2_msg_->header.stamp = now();
    // pc2_msg_->is_dense = false;
    // point_cloud_pub_->publish(*pc2_msg_);
  }

  float GET_R(float x, float y)
  {
    return sqrt(x * x + y * y);
  }
  float GET_THETA(float x, float y)
  {
    float temp_res;
    if ((x != 0))
    {
      temp_res = atan(y / x);
    }
    else
    {
      if (y >= 0)
      {
        temp_res = M_PI / 2;
      }
      else
      {
        temp_res = -M_PI / 2;
      }
    }
    if (temp_res > 0)
    {
      if (y < 0)
      {
        temp_res -= M_PI;
      }
    }
    else if (temp_res < 0)
    {
      if (x < 0)
      {
        temp_res += M_PI;
      }
    }
    // RCLCPP_INFO(this->get_logger(), "x: '%f', y: '%f', a: '%f'", x, y, temp_res);

    return temp_res;
  }
  float interpolate(float angle_1, float angle_2, float magnitude_1, float magnitude_2, float current_angle)
  {
    return (magnitude_1 + current_angle * ((magnitude_2 - magnitude_1) / (angle_2 - angle_1)));
  }
  void initialize_params()
  {
    this->declare_parameter("pointCloudTopic", "base/custom_cloud");
    this->declare_parameter("pointCloutFrameId", "laser");

    this->declare_parameter("scanTopic1", "lidar_front_right/scan");
    this->declare_parameter("laser1XOff", -0.45);
    this->declare_parameter("laser1YOff", 0.24);
    this->declare_parameter("laser1ZOff", 0.0);
    this->declare_parameter("laser1Alpha", 45.0);
    this->declare_parameter("laser1AngleMin", -181.0);
    this->declare_parameter("laser1AngleMax", 181.0);
    this->declare_parameter("laser1R", 255);
    this->declare_parameter("laser1G", 0);
    this->declare_parameter("laser1B", 0);
    this->declare_parameter("show1", true);
    this->declare_parameter("flip1", false);
    this->declare_parameter("inverse1", false);

    this->declare_parameter("scanTopic2", "lidar_rear_left/scan");
    this->declare_parameter("laser2XOff", 0.315);
    this->declare_parameter("laser2YOff", -0.24);
    this->declare_parameter("laser2ZOff", 0.0);
    this->declare_parameter("laser2Alpha", 225.0);
    this->declare_parameter("laser2AngleMin", -181.0);
    this->declare_parameter("laser2AngleMax", 181.0);
    this->declare_parameter("laser2R", 0);
    this->declare_parameter("laser2G", 0);
    this->declare_parameter("laser2B", 255);
    this->declare_parameter("show2", true);
    this->declare_parameter("flip2", false);
    this->declare_parameter("inverse2", false);
  }
  void refresh_params()
  {
    this->get_parameter_or<std::string>("pointCloudTopic", cloudTopic_, "pointCloud");
    this->get_parameter_or<std::string>("pointCloutFrameId", cloudFrameId_, "laser");
    this->get_parameter_or<std::string>("scanTopic1", topic1_, "lidar_front_right/scan");
    this->get_parameter_or<float>("laser1XOff", laser1XOff_, 0.0);
    this->get_parameter_or<float>("laser1YOff", laser1YOff_, 0.0);
    this->get_parameter_or<float>("laser1ZOff", laser1ZOff_, 0.0);
    this->get_parameter_or<float>("laser1Alpha", laser1Alpha_, 0.0);
    this->get_parameter_or<float>("laser1AngleMin", laser1AngleMin_, -181.0);
    this->get_parameter_or<float>("laser1AngleMax", laser1AngleMax_, 181.0);
    this->get_parameter_or<uint8_t>("laser1R", laser1R_, 0);
    this->get_parameter_or<uint8_t>("laser1G", laser1G_, 0);
    this->get_parameter_or<uint8_t>("laser1B", laser1B_, 0);
    this->get_parameter_or<bool>("show1", show1_, true);
    this->get_parameter_or<bool>("flip1", flip1_, false);
    this->get_parameter_or<bool>("inverse1", inverse1_, false);
    
    this->get_parameter_or<std::string>("scanTopic2", topic2_, "lidar_rear_left/scan");
    this->get_parameter_or<float>("laser2XOff", laser2XOff_, 0.0);
    this->get_parameter_or<float>("laser2YOff", laser2YOff_, 0.0);
    this->get_parameter_or<float>("laser2ZOff", laser2ZOff_, 0.0);
    this->get_parameter_or<float>("laser2Alpha", laser2Alpha_, 0.0);
    this->get_parameter_or<float>("laser2AngleMin", laser2AngleMin_, -181.0);
    this->get_parameter_or<float>("laser2AngleMax", laser2AngleMax_, 181.0);
    this->get_parameter_or<uint8_t>("laser2R", laser2R_, 0);
    this->get_parameter_or<uint8_t>("laser2G", laser2G_, 0);
    this->get_parameter_or<uint8_t>("laser2B", laser2B_, 0);
    this->get_parameter_or<bool>("show2", show2_, false);
    this->get_parameter_or<bool>("flip2", flip2_, false);
    this->get_parameter_or<bool>("inverse2", inverse2_, false);
  }
  
  std::string topic1_, topic2_, cloudTopic_, cloudFrameId_;
  bool show1_, show2_, flip1_, flip2_, inverse1_, inverse2_, is_lifted_ = false;
  float laser1XOff_, laser1YOff_, laser1ZOff_, laser1Alpha_, laser1AngleMin_, laser1AngleMax_;
  uint8_t laser1R_, laser1G_, laser1B_;

  float laser2XOff_, laser2YOff_, laser2ZOff_, laser2Alpha_, laser2AngleMin_, laser2AngleMax_;
  uint8_t laser2R_, laser2G_, laser2B_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub1_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub2_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lift_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;

  sensor_msgs::msg::LaserScan::SharedPtr laser1_;
  sensor_msgs::msg::LaserScan::SharedPtr laser2_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scanMerger>());
  rclcpp::shutdown();
  return 0;
}
