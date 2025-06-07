/**
 * @file main.cpp
 * @author LDRobot (support@ldrobot.com)
 * @brief  main process App
 *         This code is only applicable to LDROBOT LiDAR LD06 products 
 * sold by Shenzhen LDROBOT Co., LTD    
 * @version 0.1
 * @date 2021-10-28
 *
 * @copyright Copyright (c) 2021  SHENZHEN LDROBOT CO., LTD. All rights
 * reserved.
 * Licensed under the MIT License (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License in the file LICENSE
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ros2_api.h"
#include "ldlidar_driver.h"

void ToLaserscanMessagePublish(ldlidar::Points2D& src, double lidar_spin_freq, double scan_time,
  rclcpp::Time scan_start_time, LaserScanSetting& setting,
  rclcpp::Node::SharedPtr& node, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& lidarpub);

uint64_t GetSystemTimeStamp(void);

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("LD19");

  std::string product_name, topic_name, port_name;
  int serial_port_baudrate;
  ldlidar::LDType type_name;
  LaserScanSetting setting;

  setting.frame_id = "base_laser";
  setting.laser_scan_dir = true;
  setting.enable_angle_crop_func = false;
  setting.angle_crop_min = 0.0;
  setting.angle_crop_max = 0.0;

  // Declare parameters
  node->declare_parameter<std::string>("product_name", product_name);
  node->declare_parameter<std::string>("topic_name", topic_name);
  node->declare_parameter<std::string>("frame_id", setting.frame_id);
  node->declare_parameter<std::string>("port_name", port_name);
  node->declare_parameter<int>("port_baudrate", serial_port_baudrate);
  node->declare_parameter<bool>("laser_scan_dir", setting.laser_scan_dir);
  node->declare_parameter<bool>("enable_angle_crop_func", setting.enable_angle_crop_func);
  node->declare_parameter<double>("angle_crop_min", setting.angle_crop_min);
  node->declare_parameter<double>("angle_crop_max", setting.angle_crop_max);

  // Get parameters
  node->get_parameter("product_name", product_name);
  node->get_parameter("topic_name", topic_name);
  node->get_parameter("frame_id", setting.frame_id);
  node->get_parameter("port_name", port_name);
  node->get_parameter("port_baudrate", serial_port_baudrate);
  node->get_parameter("laser_scan_dir", setting.laser_scan_dir);
  node->get_parameter("enable_angle_crop_func", setting.enable_angle_crop_func);
  node->get_parameter("angle_crop_min", setting.angle_crop_min);
  node->get_parameter("angle_crop_max", setting.angle_crop_max);

  ldlidar::LDLidarDriver* ldlidarnode = new ldlidar::LDLidarDriver();

  RCLCPP_INFO(node->get_logger(), "LDLiDAR SDK Pack Version is: %s", ldlidarnode->GetLidarSdkVersionNumber().c_str());

  if (product_name == "LDLiDAR_LD06") {
    type_name = ldlidar::LDType::LD_06;
  } else if (product_name == "LDLiDAR_LD19") {
    type_name = ldlidar::LDType::LD_19;
  } else {
    RCLCPP_ERROR(node->get_logger(), "Error, input <product_name> is illegal.");
    exit(EXIT_FAILURE);
  }

  ldlidarnode->RegisterGetTimestampFunctional(std::bind(&GetSystemTimeStamp));
  ldlidarnode->EnableFilterAlgorithnmProcess(true);

  if (!ldlidarnode->Start(type_name, port_name, serial_port_baudrate, ldlidar::COMM_SERIAL_MODE)) {
    RCLCPP_ERROR(node->get_logger(), "ldlidar node start is fail");
    exit(EXIT_FAILURE);
  }

  if (!ldlidarnode->WaitLidarCommConnect(3000)) {
    RCLCPP_ERROR(node->get_logger(), "ldlidar communication is abnormal.");
    exit(EXIT_FAILURE);
  }

  auto publisher = node->create_publisher<sensor_msgs::msg::LaserScan>(topic_name, 10);
  rclcpp::WallRate r(10); // 10 Hz

  ldlidar::Points2D laser_scan_points;
  double lidar_scan_freq;
  rclcpp::Time last_scan_start_time;
  bool first_scan = true;

  RCLCPP_INFO(node->get_logger(), "Publishing ldlidar scan data...");
  while (rclcpp::ok()) {
    switch (ldlidarnode->GetLaserScanData(laser_scan_points, 1500)) {
      case ldlidar::LidarStatus::NORMAL: {
        ldlidarnode->GetLidarScanFreq(lidar_scan_freq);
        rclcpp::Time current_scan_time = node->now();
        double scan_time = 0.0;
        if (!first_scan) {
          scan_time = (current_scan_time - last_scan_start_time).seconds();
        } else {
          first_scan = false;
        }

        last_scan_start_time = current_scan_time;

        ToLaserscanMessagePublish(laser_scan_points, lidar_scan_freq, scan_time, current_scan_time,
                                  setting, node, publisher);
        break;
      }
      case ldlidar::LidarStatus::DATA_TIME_OUT:
        RCLCPP_ERROR(node->get_logger(), "get ldlidar data is time out, please check your lidar device.");
        break;
      case ldlidar::LidarStatus::DATA_WAIT:
      default:
        break;
    }

    rclcpp::spin_some(node);
    r.sleep();
  }

  ldlidarnode->Stop();
  delete ldlidarnode;
  ldlidarnode = nullptr;

  RCLCPP_INFO(node->get_logger(), "ldlidar_published is end");
  rclcpp::shutdown();
  return 0;
}

void ToLaserscanMessagePublish(ldlidar::Points2D& src, double lidar_spin_freq, double scan_time,
  rclcpp::Time scan_start_time, LaserScanSetting& setting,
  rclcpp::Node::SharedPtr& node, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& lidarpub) {
  
  float angle_min = 0;
  float angle_max = 2 * M_PI;
  float range_min = 0.02;
  float range_max = 12.0;
  int beam_size = static_cast<int>(src.size());

  if (beam_size <= 1) return;

  float angle_increment = (angle_max - angle_min) / (float)(beam_size - 1);

  sensor_msgs::msg::LaserScan output;
  output.header.stamp = scan_start_time;
  output.header.frame_id = setting.frame_id;
  output.angle_min = angle_min;
  output.angle_max = angle_max;
  output.range_min = range_min;
  output.range_max = range_max;
  output.angle_increment = angle_increment;
  output.time_increment = scan_time / (beam_size - 1);
  output.scan_time = scan_time;

  output.ranges.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
  output.intensities.assign(beam_size, std::numeric_limits<float>::quiet_NaN());

  for (const auto& point : src) {
    float range = point.distance / 1000.f;
    float intensity = point.intensity;
    float dir_angle = point.angle;

    if (point.distance == 0 && point.intensity == 0) {
      range = std::numeric_limits<float>::quiet_NaN();
      intensity = std::numeric_limits<float>::quiet_NaN();
    }

    if (setting.enable_angle_crop_func &&
        (dir_angle >= setting.angle_crop_min && dir_angle <= setting.angle_crop_max)) {
      range = std::numeric_limits<float>::quiet_NaN();
      intensity = std::numeric_limits<float>::quiet_NaN();
    }

    float angle = ANGLE_TO_RADIAN(dir_angle);
    int index = static_cast<int>((angle - angle_min) / angle_increment);
    if (index < 0 || index >= beam_size) continue;

    if (setting.laser_scan_dir) {
      int index_anticlockwise = beam_size - index - 1;
      if (std::isnan(output.ranges[index_anticlockwise]) || range < output.ranges[index_anticlockwise]) {
        output.ranges[index_anticlockwise] = range;
        output.intensities[index_anticlockwise] = intensity;
      }
    } else {
      if (std::isnan(output.ranges[index]) || range < output.ranges[index]) {
        output.ranges[index] = range;
        output.intensities[index] = intensity;
      }
    }
  }

  lidarpub->publish(output);
}

uint64_t GetSystemTimeStamp(void) {
  auto tp = std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
  return static_cast<uint64_t>(tp.time_since_epoch().count());
}


/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/