/**
 * @file lane_keeping_system.cpp
 * @author Seungho Hyeong (slkumquat@gmail.com)
 * @brief Lane Keeping System Class source file
 * @version 1.0
 * @date 2023-01-19
 */
#include "lane_keeping_system/lane_keeping_system.h"

namespace xycar {
LaneKeepingSystem::LaneKeepingSystem() {
  std::string config_path;
  nh_.getParam("config_path", config_path);
  // std::cout << config_path << std::endl;
  YAML::Node config = YAML::LoadFile(config_path);
  // std::cout << config << std::endl;

  float p_gain, i_gain, d_gain;
  int sample_size;
  pid_ptr_ = new PID(config["PID"]["P_GAIN"].as<float>(),
                     config["PID"]["I_GAIN"].as<float>(),
                     config["PID"]["D_GAIN"].as<float>());
  ma_filter_ptr_ = new MovingAverageFilter(
    config["MOVING_AVERAGE_FILTER"]["SAMPLE_SIZE"].as<int>());
  hough_transform_lane_detector_ptr_ = new HoughTransformLaneDetector(config);
  setParams(config);

  pub_ = nh_.advertise<xycar_msgs::xycar_motor>(pub_topic_name_, queue_size_);
  sub_image_ = nh_.subscribe(
    sub_image_topic_name_, queue_size_, &LaneKeepingSystem::imageCallback, this);
  sub_yolo_ = nh_.subscribe(
    sub_yolo_topic_name_, queue_size_, &LaneKeepingSystem::yoloCallback, this);
}

void LaneKeepingSystem::setParams(const YAML::Node &config) {
  pub_topic_name_ = config["TOPIC"]["PUB_NAME"].as<std::string>();
  sub_image_topic_name_ = config["TOPIC"]["SUB_IMAGE_NAME"].as<std::string>();
  sub_yolo_topic_name_ = config["TOPIC"]["SUB_YOLO_NAME"].as<std::string>();
  queue_size_ = config["TOPIC"]["QUEUE_SIZE"].as<int>();

  xycar_speed_ = config["XYCAR"]["START_SPEED"].as<float>();
  xycar_max_speed_ = config["XYCAR"]["MAX_SPEED"].as<float>();
  xycar_min_speed_ = config["XYCAR"]["MIN_SPEED"].as<float>();
  xycar_speed_control_threshold_ =
    config["XYCAR"]["SPEED_CONTROL_THRESHOLD"].as<float>();
  acceleration_step_ = config["XYCAR"]["ACCELERATION_STEP"].as<float>();
  deceleration_step_ = config["XYCAR"]["DECELERATION_STEP"].as<float>();
  choose_lane_threshold_ = config["XYCAR"]["CHOOSE_LANE_THRESHOLD"].as<float>();
  error_multiplication_ = config["XYCAR"]["ERROR_MULTIPLICATION"].as<float>();
  lane_distance_ = config["XYCAR"]["LANE_DISTANCE"].as<int>();

  keep_stop_time_ = config["STOPLINE"]["KEEP_STOP_TIME"].as<float>();
  ignore_stop_time_ = config["STOPLINE"]["IGNORE_STOP_TIME"].as<float>();
  
  reset_time_ = config["TRAFFICSIGN"]["RESET_TIME"].as<float>();
  
  debug_ = config["DEBUG"].as<bool>();
}

LaneKeepingSystem::~LaneKeepingSystem() {
  delete ma_filter_ptr_;
  delete pid_ptr_;
  delete hough_transform_lane_detector_ptr_;
}

void LaneKeepingSystem::run() {
  int lpos, rpos, error, ma_mpos;
  ros::Rate rate(30);
  while (ros::ok()) {
    ros::spinOnce();
    if (frame_.empty()) {
      continue;
    }
    time_ = ros::Time::now().toSec();

    std::tie(lpos, rpos) =
      hough_transform_lane_detector_ptr_->getLanePosition(frame_);
    
    // Don't see lane
    if (lpos == -1) {
      lpos = previous_lpos_;
    }
    if (rpos == -1) {
      rpos = previous_rpos_;
    }

    // left or right turn
    if (is_left_sign_ == true ||
        steering_angle_ < -1 * choose_lane_threshold_) {
      rpos = lpos + lane_distance_;
    }
    else if (is_right_sign_ == true ||
            steering_angle_ > choose_lane_threshold_) {
      lpos = rpos - lane_distance_;
    }

    // Detect Stop line
    bool is_stopline = hough_transform_lane_detector_ptr_->detectStopline(frame_);
    if (is_stopline == true &&
        (is_stop_sign_ == true || is_crosswalk_sign_ == true)) {
      is_stop_ = true;
    }
    else {
      is_stop_ = false;
    }

    ma_filter_ptr_->addSample((lpos + rpos) / 2);
    ma_mpos = ma_filter_ptr_->getWeightedMovingAverage();
    error = ma_mpos - frame_.cols / 2;
    steering_angle_ = std::max(-(float)kXycarSteeringAngleLimit,
                              std::min(pid_ptr_->getControlOutput(error * error_multiplication_),
                                       (float)kXycarSteeringAngleLimit));

    speed_control();
    drive(steering_angle_);
    
    // Save previous lane position
    previous_lpos_ = lpos;
    previous_rpos_ = rpos;

    // Reset Traffic sign
    if (is_traffic_sign_reset_ == false) {
      traffic_sign_reset_time_ = time_;
      is_traffic_sign_reset_ = true;
    }
    else {
      if (time_ - traffic_sign_reset_time_ > reset_time_) {
        TrafficSignReset();
        traffic_sign_reset_time_ = time_;
      }
    }

    if (debug_) {
      std::cout << "lpos: " << lpos << ", rpos: " << rpos << ", mpos: " << ma_mpos << std::endl;
      hough_transform_lane_detector_ptr_->draw_rectangles(lpos, rpos, ma_mpos);
      cv::imshow("debug",
                 *(hough_transform_lane_detector_ptr_->getDebugFrame()));
      cv::waitKey(1);
    }
    // rate.sleep();
  }
}

void LaneKeepingSystem::imageCallback(const sensor_msgs::Image &msg) {
  cv::Mat src = cv::Mat(msg.height,
                        msg.width,
                        CV_8UC3,
                        const_cast<uint8_t *>(&msg.data[0]),
                        msg.step);
  cv::cvtColor(src, frame_, cv::COLOR_RGB2BGR);
}

void LaneKeepingSystem::yoloCallback(const yolov3_trt_ros::BoundingBoxes& msg) {
  for (auto& box : msg.bounding_boxes) {
    switch(box.id) {
      case 0:
        is_left_sign_ = true;
        break;
      case 1:
        is_right_sign_ = true;
        break;
      case 2:
        is_stop_sign_ = true;
        break;
      case 3:
        is_crosswalk_sign_ = true;
        break;
      case 5:
        is_traffic_light_ = true;
        break;
      default:
        std::cout << "Not a set option" << std::endl;
    }
  }
}

void LaneKeepingSystem::TrafficSignReset() {
  // Traffic light
  is_traffic_light_ = false;
  is_red_ = false;
  is_green_ = false;
  is_yello_ = false;

  // Traffic sign
  is_left_sign_ = false;
  is_right_sign_ = false;
  is_stop_sign_ = false;
  is_crosswalk_sign_ = false;
}

void LaneKeepingSystem::speed_control() {
  if (xycar_speed_ < xycar_max_speed_) {
      xycar_speed_ += acceleration_step_;
  }
  if (is_stop_ == true) {
    if (is_stopping_ == false && is_stop_end_ == true) {
      stop_time_ = time_;
      is_stopping_ = true;
      is_stop_end_ = false;
      xycar_speed_ = 0.0;
    }
  }
  if (is_stopping_ == true) {
    if ((time_ - stop_time_ > keep_stop_time_) &&
        (time_ - stop_time_ < keep_stop_time_ + ignore_stop_time_)) {
      is_stopping_ = false;
    }
    else {
      xycar_speed_ = 0.0;
    }
  }
  else if (is_stop_end_ == false) {
    if (time_ - stop_time_ >= keep_stop_time_ + ignore_stop_time_) {
      is_stop_end_ = true;
      stop_time_ = time_;
    }
  }
}

void LaneKeepingSystem::drive(float steering_angle) {
  xycar_msgs::xycar_motor motor_msg;
  motor_msg.angle = std::round(steering_angle);
  motor_msg.speed = std::round(xycar_speed_);

  pub_.publish(motor_msg);
}
}  // namespace xycar
