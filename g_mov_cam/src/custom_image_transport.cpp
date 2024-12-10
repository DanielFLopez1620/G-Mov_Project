/**
 * Based on the iamge_transport examples of:
 * 
 * https://github.com/ros-perception/image_transport_tutorials/tree/humble
 *
 * LICENSE:
 *
 * Copyright 2021, Open Source Robotics Foundation, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// -------------------------- REQUIRED STD LIBRARIES --------------------------
#include <sstream>  // String stream library 

// -------------------------- CV RELATED LIBRARIES ----------------------------
#include "cv_bridge/cv_bridge.h"  // ROS CV Bridge
#include "opencv2/core/mat.hpp"   // Matrix/Image datas for OpenCV
#include "opencv2/videoio.hpp"    // Video input/ouput library

// ------------------------- ROS 2 REQUIRED HEADERS ---------------------------
#include "rclcpp/rclcpp.hpp"      // ROS 2 Client library for C++

// ------------------------- ROS 2 MSGS IMPORTS -------------------------------
#include "image_transport/image_transport.hpp"  // Custom image transport msgs
#include "std_msgs/msg/header.hpp"              // Header standard msgs

// --------------------------- MAIN IMPLEMENTATION ----------------------------
int main(int argc, char ** argv)
{
	// Check if video source has been passed as a parameter (, for example, 0)
	if (argv[1] == NULL) 
	{
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No video source provided");
		return 1;
	}

	// Initialize node
	rclcpp::init(argc, argv);
	rclcpp::NodeOptions options;
	auto node = rclcpp::Node::make_shared("image_publisher", options);

	// Instance Image Tranport and create the publisher
	image_transport::ImageTransport it(node);
	image_transport::Publisher pub = it.advertise("/raw_image/compressed", 1);

	// Convert the command line parameter index for the video device to an integer
	std::istringstream video_sourceCmd(argv[1]);
	int video_source;
	if (!(video_sourceCmd >> video_source)) 
	{
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Video source must be an integer");
		return 1;
	}

	// Read input camera based on Video4Linux apt package
	cv::VideoCapture cap(video_source, cv::CAP_V4L2);

	// Adapt reading in terms of size and speed
	cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
	cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);  
	cap.set(cv::CAP_PROP_FPS, 5); 

	// Check that video can be accessed
	if (!cap.isOpened()) 
	{
		RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not open video source");
		return 1;
	}

	// Create frame
	cv::Mat frame;

	// Instance image msg
	std_msgs::msg::Header hdr;
	sensor_msgs::msg::Image::SharedPtr msg;

	rclcpp::WallRate loop_rate(5);

	// Loop for publishing video
	while (rclcpp::ok()) 
	{
		// Read frame
		cap >> frame;

		// Check frame
		if (frame.empty()) 
		{
			RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Captured empty frame");
			continue;
		}

		// update content of message
		hdr.stamp = node->now();
		msg = cv_bridge::CvImage(hdr, "bgr8", frame).toImageMsg();
		
		// Publish msg
		pub.publish(msg);

		// Spin and add a dealy
		rclcpp::spin_some(node);
		loop_rate.sleep();
	}

	// Close and shutdown program
	cap.release();
	rclcpp::shutdown();
	return 0;
}
