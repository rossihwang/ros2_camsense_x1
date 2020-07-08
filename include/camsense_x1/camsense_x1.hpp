// Copyright <2020> [Copyright rossihwang@gmail.com]

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <serial/serial.h>

class CamsenseX1 : public rclcpp::Node {
 public:
  CamsenseX1(const std::string &name, rclcpp::NodeOptions const &options);
  ~CamsenseX1();

 private:
  enum class State {
    SYNC1,
    SYNC2,
    SYNC3,
    SYNC4,
    SPEED,
    START,
    DATA,
    END,
    CRC,
    PACK_FIN
  };
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  std::shared_ptr<serial::Serial> serial_ptr_;
  std::string port_;
  int baud_;
  State state_;
  uint8_t buffer_[2];
  uint8_t frame_data_[24];
  std::thread thread_;
  std::atomic<bool> canceled_;
  uint16_t speed_;
  float start_angle_;
  float end_angle_;
  std::vector<float> ranges_;
  std::vector<float> intensities_;

  void parse();
  void reset_parse();
};