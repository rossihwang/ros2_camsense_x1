// Copyright <2020> [Copyright rossihwang@gmail.com]

#include <camsense_x1/camsense_x1.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <cmath>

constexpr uint8_t kSync1 = 0x55;
constexpr uint8_t kSync2 = 0xaa;
constexpr uint8_t kSync3 = 0x03;
constexpr uint8_t kSync4 = 0x08; 

CamsenseX1::CamsenseX1(const std::string &name, rclcpp::NodeOptions const &options)
  : Node(name, options),
    frame_id_("scan"),
    port_("/dev/ttyUSB0"),
    baud_(115200),
    angle_offset_(0),
    state_(State::SYNC1),
    canceled_(false),
    speed_(0),
    start_angle_(0.0),
    end_angle_(0.0),
    rate_(std::chrono::milliseconds(32)) {
  scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
  serial_ptr_ = std::make_shared<serial::Serial>(port_, baud_, serial::Timeout::simpleTimeout(1000));
  
  RCLCPP_INFO(this->get_logger(), "%s is open(%d)", port_.c_str(), serial_ptr_->isOpen());
  
  ranges_.resize(360);
  intensities_.resize(360);

  create_parameter();
  reset_data();

  thread_ = std::thread{[this]() -> void {
    while (rclcpp::ok() && !canceled_.load()) {
      parse();
    }
  }};
}

CamsenseX1::~CamsenseX1() {
  canceled_.store(true);
  if (thread_.joinable()) {
    thread_.join();
  }
}

void CamsenseX1::parse() {
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
        angle_res = (end_angle_ + 360 - start_angle_) / 7.0;
      } else {
        angle_res = (end_angle_ - start_angle_) / 7.0;
      }
      for (int i = 0; i < 8; ++i) {
        int j = 3 * i;
        uint16_t range = static_cast<uint16_t>(frame_data_[j+1]) << 8 | frame_data_[j];
        uint8_t intensity = frame_data_[j+2];
        double measured_angle = start_angle_ + angle_res * i - angle_offset_;
        int angle_index = std::round(measured_angle);
        angle_index %= 360;
        angle_index = 359 - angle_index;
        if (range == 0x8000) {
          range = 8000;  // maximum range in mm
        }
        ranges_[angle_index] = static_cast<float>(range) / 1000.0;
        intensities_[angle_index] = static_cast<float>(intensity);
      }
      if (end_angle_ < start_angle_) {
        // RCLCPP_INFO(this->get_logger(), "publish");
        sensor_msgs::msg::LaserScan message;
        message.header.stamp = now();
        message.header.frame_id = frame_id_;
        message.angle_increment = (2.0 * M_PI) / 360.0;
        message.angle_min = 0.0;
        message.angle_max = 2.0 * M_PI - message.angle_increment;
        message.scan_time = 0.001;
        message.range_min = 0.08;  // camsense x1 spec
        message.range_max = 8;    // camsense x1 spec
        message.ranges = ranges_;
        message.intensities = intensities_;
        scan_pub_->publish(message);
        reset_data();
        rate_.sleep();
      }
      state_ = State::SYNC1;
      break;
    default:
      break;
  }
}

void CamsenseX1::reset_data() {
  for (int i = 0; i < 360; ++i) {
    ranges_[i] = 8;
    intensities_[i] = 0;
  }
}

void CamsenseX1::create_parameter() {
  frame_id_ = declare_parameter<std::string>("frame_id", "scan");
  port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
  baud_ = declare_parameter<int>("baud", 115200);
  angle_offset_ = declare_parameter<int>("angle_offset", 0);

  set_on_parameters_set_callback (
    [this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult {
      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;
      for (auto const &p : parameters) {
        result.successful &= handle_parameter(p);
      }
      return result;
    });
}

bool CamsenseX1::handle_parameter(rclcpp::Parameter const &param) {
  if (param.get_name() == "frame_id") {
    frame_id_ = param.as_string();
  } else if (param.get_name() == "port") {
    port_ = param.as_string();
  } else if (param.get_name() == "baud") {
    baud_ = param.as_int();
  } else if (param.get_name() == "rotation") {
    angle_offset_ = param.as_int() % 360;
  } else {
    return false;
  }

  return true;
}
