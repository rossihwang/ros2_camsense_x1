// Copyright <2020> [Copyright rossihwang@gmail.com]

#include <camsense_x1/camsense_x1.hpp>
#include <cmath>

constexpr uint8_t kSync1 = 0x55;
constexpr uint8_t kSync2 = 0xaa;
constexpr uint8_t kSync3 = 0x03;
constexpr uint8_t kSync4 = 0x08; 

CamsenseX1::CamsenseX1(const std::string &name, rclcpp::NodeOptions const &options)
  : Node(name, options),
    port_("/dev/ttyUSB0"),
    baud_(115200),
    state_(State::SYNC1),
    canceled_(false),
    speed_(0),
    start_angle_(0.0),
    end_angle_(0.0) {
  scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
  serial_ptr_ = std::make_shared<serial::Serial>(port_, baud_, serial::Timeout::simpleTimeout(1000));
  
  RCLCPP_INFO(this->get_logger(), "%s is open(%d)", port_.c_str(), serial_ptr_->isOpen());
  
  ranges_.resize(360);
  intensities_.resize(360);

  InitData();

  thread_ = std::thread{[this]() -> void {
    rclcpp::Rate rate(std::chrono::milliseconds(1));
    while (rclcpp::ok() && !canceled_.load()) {
      Parse();
      // rate.sleep();
    }
  }};
}

CamsenseX1::~CamsenseX1() {
  canceled_.store(true);
  if (thread_.joinable()) {
    thread_.join();
  }
}

void CamsenseX1::Parse() {
  switch (state_) {
    case State::SYNC1:
      serial_ptr_->read(buffer_, 1);
      if (buffer_[0] == kSync1) {
        state_ = State::SYNC2;
      }
      break;
    case State::SYNC2:
      serial_ptr_->read(buffer_, 1);
      if (buffer_[0] == kSync2) {
        state_ = State::SYNC3;
      } else {
        state_ = State::SYNC1;
      }
      break;
    case State::SYNC3:
      serial_ptr_->read(buffer_, 1);
      if (buffer_[0] == kSync3) {
        state_ = State::SYNC4;
      } else {
        state_ = State::SYNC1;
      }
      break;
    case State::SYNC4:
      serial_ptr_->read(buffer_, 1);
      if (buffer_[0] == kSync4) {
        state_ = State::SPEED;
      } else {
        state_ = State::SYNC1;
      }
      break;
    case State::SPEED:
      serial_ptr_->read(buffer_, 2);
      speed_ = static_cast<uint16_t>(buffer_[1]) << 8 | buffer_[0];
      // RCLCPP_INFO(this->get_logger(), "speed: %d ", speed_);
      state_ = State::START;
      break;
    case State::START: {
      uint16_t count;
      serial_ptr_->read(buffer_, 2);
      count = static_cast<uint16_t>(buffer_[1]) << 8 | buffer_[0];
      start_angle_ = count / 64.0 - 640;
      state_ = State::DATA;
      break;
    }
    case State::DATA:
      serial_ptr_->read(frame_data_, 24);
      state_ = State::END;
      break;
    case State::END: {
      uint16_t count;
      serial_ptr_->read(buffer_, 2);
      count = static_cast<uint16_t>(buffer_[1]) << 8 | buffer_[0];
      end_angle_ = count / 64.0 - 640;
      state_ = State::CRC;
      break;
    }
    case State::CRC:
      serial_ptr_->read(buffer_, 2);
      // TODO: validate crc
      float angle_res;
      if (end_angle_ < start_angle_) {
        angle_res = (end_angle_ + 360 - start_angle_) / 8.0;
      } else {
        angle_res = (end_angle_ - start_angle_) / 8.0;
      }
      for (int i = 0; i < 8; ++i) {
        int j = 3 * i;
        uint16_t range = static_cast<uint16_t>(frame_data_[j+1]) << 8 | frame_data_[j];
        uint8_t intensity = frame_data_[j+2];
        int angle = std::round(start_angle_ + angle_res * i);
        angle = 360 - angle;
        // RCLCPP_INFO(this->get_logger(), "angle: %d ", angle);
        if (range == 0x8000) {
          range = 8000;
        }
        if (ranges_[angle] == 8) {
          ranges_[angle] = static_cast<float>(range) / 1000.0;
        } else {
          ranges_[angle] = (ranges_[angle] + (static_cast<float>(range))/1000.0) / 2.0;
        }
        
        if (intensities_[angle] == 0) {
          intensities_[angle] = static_cast<float>(intensity);
        } else {
          intensities_[angle] = (intensities_[angle] + static_cast<float>(intensity)) / 2.0;
        }
        
      }
      if (end_angle_ < start_angle_) {
        // RCLCPP_INFO(this->get_logger(), "publish");
        sensor_msgs::msg::LaserScan message;
        message.header.stamp = now();
        message.header.frame_id = "scan";
        message.angle_increment = (2.0 * M_PI) / 360.0;
        message.angle_min = 0.0;
        message.angle_max = 2.0 * M_PI - message.angle_increment;
        message.scan_time = 0.001;
        message.range_min = 0.08;
        message.range_max = 8;
        message.ranges = ranges_;
        message.intensities = intensities_;
        scan_pub_->publish(message);
        InitData();
      }
      state_ = State::SYNC1;
      break;
    default:
      break;
  }
}

void CamsenseX1::InitData() {
  for (int i = 0; i < 360; ++i) {
    ranges_[i] = 8;
    intensities_[i] = 0;
  }
}

