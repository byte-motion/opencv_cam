#include "opencv_cam/opencv_cam_node.hpp"

#include <iostream>

#include "camera_calibration_parsers/parse.hpp"

namespace opencv_cam
{

  std::string mat_type2encoding(int mat_type)
  {
    switch (mat_type)
    {
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "bgr8";
    case CV_16SC1:
      return "mono16";
    case CV_8UC4:
      return "rgba8";
    default:
      throw std::runtime_error("unsupported encoding type");
    }
  }

  OpencvCamNode::OpencvCamNode(const rclcpp::NodeOptions &options) : Node("opencv_cam", options),
                                                                     canceled_(false),
                                                                     publish_next_(true)
  {
    RCLCPP_INFO(get_logger(), "use_intra_process_comms=%d", options.use_intra_process_comms());

    // Initialize parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
    CXT_MACRO_INIT_PARAMETERS(OPENCV_CAM_ALL_PARAMS, validate_parameters)

    // Register for parameter changed. NOTE at this point nothing is done when parameters change.
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), cxt_, OPENCV_CAM_ALL_PARAMS, validate_parameters)

    // Log the current parameters
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_SORTED_PARAMETER(cxt_, n, t, d)
    CXT_MACRO_LOG_SORTED_PARAMETERS(RCLCPP_INFO, get_logger(), "opencv_cam Parameters", OPENCV_CAM_ALL_PARAMS)

    // Check that all command line parameters are registered
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_CHECK_CMDLINE_PARAMETER(n, t, d)
    CXT_MACRO_CHECK_CMDLINE_PARAMETERS((*this), OPENCV_CAM_ALL_PARAMS)

    RCLCPP_INFO(get_logger(), "OpenCV version %d", CV_VERSION_MAJOR);

    double width = 0;
    double height = 0;

    // Open file or device
    if (cxt_.file_)
    {
      capture_ = std::make_shared<cv::VideoCapture>(cxt_.filename_);

      if (!capture_->isOpened())
      {
        RCLCPP_ERROR(get_logger(), "cannot open file %s", cxt_.filename_.c_str());
        return;
      }

      if (cxt_.fps_ > 0)
      {
        // Publish at the specified rate
        publish_fps_ = cxt_.fps_;
      }
      else
      {
        // Publish at the recorded rate
        publish_fps_ = static_cast<int>(capture_->get(cv::CAP_PROP_FPS));
      }

      width = capture_->get(cv::CAP_PROP_FRAME_WIDTH);
      height = capture_->get(cv::CAP_PROP_FRAME_HEIGHT);
      RCLCPP_INFO(get_logger(), "file %s open, width %g, height %g, publish fps %d",
                  cxt_.filename_.c_str(), width, height, publish_fps_);

      next_stamp_ = now();
    }
    else
    {
      capture_ = std::make_shared<cv::VideoCapture>(cxt_.index_);

      if (!capture_->isOpened())
      {
        RCLCPP_ERROR(get_logger(), "cannot open device %d", cxt_.index_);
        return;
      }

      if (cxt_.height_ > 0)
      {
        capture_->set(cv::CAP_PROP_FRAME_HEIGHT, cxt_.height_);
      }

      if (cxt_.width_ > 0)
      {
        capture_->set(cv::CAP_PROP_FRAME_WIDTH, cxt_.width_);
      }

      if (cxt_.fps_ > 0)
      {
        capture_->set(cv::CAP_PROP_FPS, cxt_.fps_);
      }

      width = capture_->get(cv::CAP_PROP_FRAME_WIDTH);
      height = capture_->get(cv::CAP_PROP_FRAME_HEIGHT);
      double fps = capture_->get(cv::CAP_PROP_FPS);
      RCLCPP_INFO(get_logger(), "device %d open, width %g, height %g, device fps %g",
                  cxt_.index_, width, height, fps);
    }

    assert(!cxt_.camera_info_path_.empty()); // readCalibration will crash if file_name is ""
    std::string camera_name;
    if (camera_calibration_parsers::readCalibration(cxt_.camera_info_path_, camera_name, camera_info_msg_))
    {
      RCLCPP_INFO(get_logger(), "got camera info for '%s'", camera_name.c_str());
      camera_info_msg_.header.frame_id = cxt_.camera_frame_id_;
      camera_info_msg_.width = width;
      camera_info_msg_.height = height;
      camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 10);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "cannot get camera info, will not publish");
      camera_info_pub_ = nullptr;
    }

    image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
    trigger_service_ = this->create_service<std_srvs::srv::Trigger>(
        "trigger_capture",
        std::bind(&OpencvCamNode::trigger_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Run loop on it's own thread
    thread_ = std::thread(std::bind(&OpencvCamNode::loop, this));

    RCLCPP_INFO(get_logger(), "start publishing");
  }

  OpencvCamNode::~OpencvCamNode()
  {
    // Stop loop
    canceled_.store(true);
    if (thread_.joinable())
    {
      thread_.join();
    }
  }

  void OpencvCamNode::validate_parameters()
  {
  }

  void OpencvCamNode::trigger_callback(
      [[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    RCLCPP_DEBUG(this->get_logger(), "Received trigger request");
    publish_next_ = true;
    response->success = true;
    response->message = "Capture triggered";
  }

  void OpencvCamNode::loop()
  {
    cv::Mat frame;
    bool single_image_mode = cxt_.file_ && capture_->get(cv::CAP_PROP_FRAME_COUNT) == 1;
    bool frame_loaded = false;
    RCLCPP_INFO(get_logger(), "Single Image Mode = %d", single_image_mode);

    while (rclcpp::ok() && !canceled_.load())
    {
      // Read a frame if not in single frame mode or if the frame hasn't been loaded yet
      if (!single_image_mode || !frame_loaded)
      {
        if (!capture_->read(frame))
        {
          if (cxt_.file_)
          {
            RCLCPP_INFO(get_logger(), "Reached EOF, looping back to start.");
            capture_->set(cv::CAP_PROP_POS_FRAMES, 0); // Loop back to start
            continue;                                  // Skip the rest of this iteration and try reading again
          }
          else
          {
            RCLCPP_INFO(get_logger(), "EOF or error reading frame, stop publishing");
            break;
          }
        }

        // Mark the frame as loaded if we're in single frame mode
        if (single_image_mode)
        {
          frame_loaded = true;
        }
      }

      auto stamp = now();

      // Prepare the image message
      sensor_msgs::msg::Image::UniquePtr image_msg(new sensor_msgs::msg::Image());

      // Convert OpenCV Mat to ROS Image
      image_msg->header.stamp = stamp;
      image_msg->header.frame_id = cxt_.camera_frame_id_;
      image_msg->height = frame.rows;
      image_msg->width = frame.cols;
      image_msg->encoding = mat_type2encoding(frame.type());
      image_msg->is_bigendian = false;
      image_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
      image_msg->data.assign(frame.datastart, frame.dataend);

#undef SHOW_ADDRESS
#ifdef SHOW_ADDRESS
      static int count = 0;
      RCLCPP_INFO(get_logger(), "%d, %p", count++, reinterpret_cast<std::uintptr_t>(image_msg.get()));
#endif

      // Publish
      if (cxt_.sync_mode_ == false || publish_next_)
      {
        image_pub_->publish(std::move(image_msg));
        if (camera_info_pub_)
        {
          camera_info_msg_.header.stamp = stamp;
          camera_info_pub_->publish(camera_info_msg_);
        }
        publish_next_ = false;
      }

      // Sleep if required
      if (publish_fps_ > 0)
      {
        using namespace std::chrono_literals;
        next_stamp_ = next_stamp_ + rclcpp::Duration{1000000000ns / publish_fps_};
        auto wait = next_stamp_ - stamp;
        if (wait.nanoseconds() > 0)
        {
          std::this_thread::sleep_for(static_cast<std::chrono::nanoseconds>(wait.nanoseconds()));
        }
      }
    }
  }

} // namespace opencv_cam

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(opencv_cam::OpencvCamNode)