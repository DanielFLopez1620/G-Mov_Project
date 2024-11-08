#include <sstream>
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "opencv2/core/mat.hpp"
#include "opencv2/videoio.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

int main(int argc, char ** argv)
{
  // Check if video source has been passed as a parameter
  if (argv[1] == NULL) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No video source provided");
    return 1;
  }

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = rclcpp::Node::make_shared("image_publisher", options);

  image_transport::ImageTransport it(node);
  image_transport::Publisher pub = it.advertise("/raw_image/compressed", 1);

  // Convert the command line parameter index for the video device to an integer
  std::istringstream video_sourceCmd(argv[1]);
  int video_source;
  if (!(video_sourceCmd >> video_source)) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Video source must be an integer");
    return 1;
  }

  cv::VideoCapture cap(video_source, cv::CAP_V4L2);
  cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);   // Set a lower width (e.g., 320 for lower resolution)
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);  // Set a lower height (e.g., 240 for lower resolution)
  cap.set(cv::CAP_PROP_FPS, 5); 
  if (!cap.isOpened()) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not open video source");
    return 1;
  }

  cv::Mat frame;
  std_msgs::msg::Header hdr;
  sensor_msgs::msg::Image::SharedPtr msg;

  rclcpp::WallRate loop_rate(5);
  while (rclcpp::ok()) {
    cap >> frame;
    if (frame.empty()) {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Captured empty frame");
      continue;
    }

    hdr.stamp = node->now();  // Update timestamp
    msg = cv_bridge::CvImage(hdr, "bgr8", frame).toImageMsg();
    pub.publish(msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  cap.release();
  rclcpp::shutdown();
  return 0;
}
