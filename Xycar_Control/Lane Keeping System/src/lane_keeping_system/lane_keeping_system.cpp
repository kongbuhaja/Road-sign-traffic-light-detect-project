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
  YAML::Node config = YAML::LoadFile(config_path);

  // Set PID
  float p_gain = config["PID"]["P_GAIN"].as<float>();
  float i_gain = config["PID"]["I_GAIN"].as<float>();
  float d_gain = config["PID"]["D_GAIN"].as<float>();
  pid_ptr_ = new PID(p_gain, i_gain, d_gain);
  
  // Set Moving Average Filter
  int sample_size = config["MOVING_AVERAGE_FILTER"]["SAMPLE_SIZE"].as<int>();				 
  ma_filter_ptr_ = new MovingAverageFilter(sample_size);
  
  
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
  
  error_multiplication_ =
    config["STEERING_CONTROL"]["ERROR_MULTIPLICATION"].as<float>();
  lane_distance_ = config["STEERING_CONTROL"]["LANE_DISTANCE"].as<int>();

  keep_stop_time_ = config["STOPLINE"]["KEEP_STOP_TIME"].as<float>();
  ignore_stop_time_ = config["STOPLINE"]["IGNORE_STOP_TIME"].as<float>();
  
  traffic_sign_detection_threshold_ =
    config["TRAFFICSIGN"]["TRAFFIC_SIGN_DETECTION_THRESHOLD"].as<int>();
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
  ros::Rate rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    if (frame_.empty()) {
      continue;
    }
    // Begin time.
    time_ = ros::Time::now().toSec();

    // Detect Left & Right Lane.
    std::tie(lpos, rpos) =
      hough_transform_lane_detector_ptr_->getLanePosition(frame_);
    
    // Lane Decision Logic.
	// Go straight if you can't see both lanes.
    if (lpos == -1 && rpos == -1) {
      if (abs(steering_angle_) < 10.0) {
        is_straight_ = true;
      }
    }
	
	// If a lane is not detected, the position of the previous lane is used.
    if (lpos == -1) {
      lpos = previous_lpos_;   
    }
    if (rpos == -1) {
      rpos = previous_rpos_;
    }
    
	// Select driving mode (use traffic sign)
    if (is_left_sign_ == true) {
      driving_mode_ = 0;
    }
    else if (is_right_sign_ == true) {
      driving_mode_ = 1;
    }
	
	// If you are turning right, look to the right lane
	// and if you are turning left, look to the left lane.
    if (driving_mode_ == 0) {
      rpos = lpos + lane_distance_;
    }
    else if (driving_mode_ == 1) {
      lpos = rpos - lane_distance_;
    }

    
    if (abs(lpos - previous_lpos_) > 50) { 
	  if (lpos_lost_time_ == -1) {
		  lpos_lost_time_ = ros::Time::now().toSec();
	  }
	  
      if (lpos_lost_time_ - time_ < 1) {
        lpos = previous_lpos_;
      }
      else {
        lpos_lost_time_ = -1;
      }
    }
    if (abs(rpos - previous_rpos_) > 50) {
	  if (rpos_lost_time_ == -1) {
		  rpos_lost_time_ = ros::Time::now().toSec();
	  }
	  
      if (rpos_lost_time_ - time_ < 1) {
        rpos = previous_rpos_;
      }
      else {
        rpos_lost_time_ = -1;
      }
    }

    // Detect Stop line.
    is_stopline_ = hough_transform_lane_detector_ptr_->detectStopline(frame_);

    // Detect Stop line && (stop sign or crosswalk sign or red light)
    if (is_stopline_ == true &&
        (is_stop_sign_ || is_crosswalk_sign_)) {
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

    if (is_straight_ == true) {
      steering_angle_ = 0.0;
    }
	
    speed_control();
    if (abs(steering_angle_ - previous_steering_angle_) > 15)
    {
      steering_angle_ = previous_steering_angle_;
    }
    drive();
    TrafficSignReset();

    // Save previous lane position
    previous_lpos_ = lpos;
    previous_rpos_ = rpos;
    is_straight_ = false;
    previous_steering_angle_ = steering_angle_;

    if (debug_) {
      std::cout << lpos << ", " << rpos << std::endl;
      hough_transform_lane_detector_ptr_->draw_rectangles(lpos, rpos, ma_mpos);
      hough_transform_lane_detector_ptr_->draw_text(driving_mode_);
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
  TrafficSign left_sign;
  TrafficSign right_sign;
  TrafficSign stop_sign;
  TrafficSign crosswalk_sign;
  TrafficSign traffic_light;
  TrafficSign bbox;

  int sign_area;
  for (auto& box : msg.bounding_boxes) {
    bbox.id = box.id;
    bbox.probability = box.probability;
    bbox.xmax = box.xmax;
    bbox.ymax = box.ymax;
    bbox.xmin = box.xmin;
    bbox.ymin = box.ymin;
    sign_area = (box.xmax - box.xmin) * (box.ymax - box.ymin);
    bbox.area = sign_area;

    if (bbox.xmin < 0) bbox.xmin = 0;
    if (bbox.ymin < 0) bbox.ymin = 0;
    if (bbox.xmax > yolo_width_) bbox.xmax = yolo_width_ - 1;
    if (bbox.ymax > yolo_width_) bbox.ymax = yolo_width_ - 1;

    if (box.id == 0) {
      if (bbox.area > traffic_sign_detection_threshold_) {
        if (left_sign.id == -1) {
          left_sign = bbox;
        }
        else {
          if (bbox.area > left_sign.area) {
            left_sign = bbox;
          }
        }
        is_left_sign_ = true;
      }
    }
    else if (box.id == 1) {
      if (bbox.area > traffic_sign_detection_threshold_) {
        if (right_sign.id == -1) {
          right_sign = bbox;
        }
        else {
          if (bbox.area > right_sign.area) {
            right_sign = bbox;
          }
        }
        is_right_sign_ = true;
      }
    }
    else if (box.id == 2) {
      if (bbox.area > traffic_sign_detection_threshold_) {
        if (stop_sign.id == -1) {
          stop_sign = bbox;
        }
        else {
          if (bbox.area > stop_sign.area) {
            stop_sign = bbox;
          }
        }
        is_stop_sign_ = true;
      }
    }
    else if (box.id == 3) {
      if (bbox.area > traffic_sign_detection_threshold_) {
        if (crosswalk_sign.id == -1) {
          crosswalk_sign = bbox;
        }
        else {
          if (bbox.area > crosswalk_sign.area) {
            crosswalk_sign = bbox;
          }
        }
        is_crosswalk_sign_ = true;
      }
    }
    else if (box.id == 5) {
      if (bbox.area > traffic_sign_detection_threshold_) {
        if (traffic_light.id == -1) {
          traffic_light = bbox;
        }
        else {
          if (bbox.area > crosswalk_sign.area) {
            traffic_light = bbox;
          }
        }
        is_traffic_light_ = true;
      }
    }
  }

  // Recognition Traffic light. (Red, Yellow, Green)
  cv::Mat image, resize_image;
  frame_.copyTo(image);
  cv::resize(image, resize_image, cv::Size(416, 416));
  if (traffic_light.id != -1) {
    cv::Mat image_;
    resize_image.copyTo(image_);

    cv::Mat trafficlight_roi = 
      image_(cv::Rect(cv::Point(traffic_light.xmin, traffic_light.ymin),
                      cv::Point(traffic_light.xmax, traffic_light.ymax)));

    cv::Mat trafficlight_rotate;
    cv::rotate(trafficlight_roi, trafficlight_rotate, cv::ROTATE_90_CLOCKWISE);
    
    int row1 = (int)(trafficlight_rotate.rows / 4);
    int row2 = (int)(trafficlight_rotate.rows / 2);
    int row3 = (int)(trafficlight_rotate.rows / 4 * 3);
    
    auto *ptr_row1 = trafficlight_rotate.ptr<uchar>(row1);
    float sum_pixel1 = 0;
    for (int col = 0; col < trafficlight_rotate.cols; col++) {
      uchar g = ptr_row1[col * 3 + 1];
      uchar r = ptr_row1[col * 3 + 2];
      sum_pixel1 += r - g;
    }

    auto *ptr_row2 = trafficlight_rotate.ptr<uchar>(row2);
    float sum_pixel2 = 0;
    for (int col = 0; col < trafficlight_rotate.cols; col++) {
      uchar g = ptr_row2[col * 3 + 1];
      uchar r = ptr_row2[col * 3 + 2];
      sum_pixel2 += r - g;
    }

    auto *ptr_row3 = trafficlight_rotate.ptr<uchar>(row3);
    float sum_pixel3 = 0;
    for (int col = 0; col < trafficlight_rotate.cols; col++) {
      uchar g = ptr_row3[col * 3 + 1];
      uchar r = ptr_row3[col * 3 + 2];
      sum_pixel3 += r - g;
    }

    float pixel_th = 0.0;
    int traffic_light_flag = 0;
    if (sum_pixel1 > pixel_th) {
      traffic_light_flag++;
    }
    else {
      traffic_light_flag--;
    }

    if (sum_pixel2 > pixel_th) {
      traffic_light_flag++;
    }
    else {
      traffic_light_flag--;
    }

    if (sum_pixel3 > pixel_th) {
      traffic_light_flag++;
    }
    else {
      traffic_light_flag--;
    }

    if (traffic_light_flag > 0) {
      is_red_ = true;
    }
    else {
      is_green_ = true;
    }
  }

  // Traffic sign Show
  if (debug_) {
    if (left_sign.id != -1) {
      cv::rectangle(resize_image,
              cv::Point(left_sign.xmin, left_sign.ymin),
              cv::Point(left_sign.xmax, left_sign.ymax),
              cv::Scalar(255, 0, 0), 2); 
      cv::putText(resize_image,
                  "Left Sign", cv::Point(left_sign.xmin, left_sign.ymin),
                  2, 0.5, cv::Scalar(255, 0, 0));
    }
    if (right_sign.id != -1) {
      cv::rectangle(resize_image,
              cv::Point(right_sign.xmin, right_sign.ymin),
              cv::Point(right_sign.xmax, right_sign.ymax),
              cv::Scalar(0, 165, 255), 2);      
      cv::putText(resize_image,
                "Right Sign", cv::Point(right_sign.xmin, right_sign.ymin),
                2, 0.5, cv::Scalar(0, 165, 255));
    }
    if (stop_sign.id != -1) {
      cv::rectangle(resize_image,
              cv::Point(stop_sign.xmin, stop_sign.ymin),
              cv::Point(stop_sign.xmax, stop_sign.ymax),
              cv::Scalar(0, 255, 255),2); 
      cv::putText(resize_image,
          "Stop Sign", cv::Point(stop_sign.xmin, stop_sign.ymin),
          2, 0.5, cv::Scalar(0, 255, 255));        
    }
    if (crosswalk_sign.id != -1) {
      cv::rectangle(resize_image,
              cv::Point(crosswalk_sign.xmin, crosswalk_sign.ymin),
              cv::Point(crosswalk_sign.xmax, crosswalk_sign.ymax),
              cv::Scalar(0, 255, 0), 2);
      cv::putText(resize_image,
          "Crosswalk Sign", cv::Point(crosswalk_sign.xmin, crosswalk_sign.ymin),
          2, 0.5, cv::Scalar(0, 255, 0));        
    }
    if (traffic_light.id != -1) {
      cv::Scalar red(0, 0, 255);
      cv::Scalar green(0, 255, 0);
      cv::Scalar yellow(0, 255, 255);
      cv::Scalar black(0, 0, 0);
      if (is_red_) {
        std::cout << "red" << std::endl;
        cv::rectangle(resize_image,
          cv::Point(traffic_light.xmin, traffic_light.ymin),
          cv::Point(traffic_light.xmax, traffic_light.ymax),
          red, 2); 
      }
      else if (is_green_) {
        cv::rectangle(resize_image,
          cv::Point(traffic_light.xmin, traffic_light.ymin),
          cv::Point(traffic_light.xmax, traffic_light.ymax),
          green, 2);
        cv::putText(resize_image,
          "Green Light", cv::Point(traffic_light.xmin, traffic_light.ymin),
          2, 0.5, cv::Scalar(0, 255, 0));   
      }
      else if (is_yellow_) {
        cv::rectangle(resize_image,
          cv::Point(traffic_light.xmin, traffic_light.ymin),
          cv::Point(traffic_light.xmax, traffic_light.ymax),
          yellow, 2);
        cv::putText(resize_image,
          "Yellow Light", cv::Point(traffic_light.xmin, traffic_light.ymin),
          2, 0.5, cv::Scalar(0, 255, 255)); 
      }
      else {
        cv::rectangle(resize_image,
          cv::Point(traffic_light.xmin, traffic_light.ymin),
          cv::Point(traffic_light.xmax, traffic_light.ymax),
          black, 2);
        cv::putText(resize_image,
          "Red Light", cv::Point(traffic_light.xmin, traffic_light.ymin),
          2, 0.5, cv::Scalar(0, 0, 255)); 
      }
    }
  }
  cv::imshow("Traffic Sign Detection", resize_image);
}

void LaneKeepingSystem::TrafficSignReset() {
  if (is_traffic_sign_reset_ == false) {
    traffic_sign_reset_time_ = time_;
    is_traffic_sign_reset_ = true;
  }
  else {
    if (time_ - traffic_sign_reset_time_ > reset_time_) {
      // Traffic light
      is_traffic_light_ = false;
      is_red_ = false;
      is_green_ = false;
      is_yellow_ = false;

      // Traffic sign
      is_left_sign_ = false;
      is_right_sign_ = false;
      is_stop_sign_ = false;
      is_crosswalk_sign_ = false;
      
      traffic_sign_reset_time_ = time_;
    }
  }
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
  if(is_red_ && is_stopline_) {
    xycar_speed_ = 0.0;
  }
}

void LaneKeepingSystem::drive() {
  xycar_msgs::xycar_motor motor_msg;
  motor_msg.angle = std::round(steering_angle_);
  motor_msg.speed = std::round(xycar_speed_);

  pub_.publish(motor_msg);
}
}  // namespace xycar
