/**
 * @file img_stack.hpp
 * @author Manu Madhu Pillai (manump@umd.edu)
 * @brief Header for Vision Stack 
 * @version 0.1
 * @date 2022-12-08
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef PANDA_CONTROLLER_INCLUDE_PANDA_CONTROLLER_IMG_STACK_HPP_ 
#define PANDA_CONTROLLER_INCLUDE_PANDA_CONTROLLER_IMG_STACK_HPP_ 
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include "panda_world/msg/block_location.hpp"
#include "panda_world/msg/multi_block_location.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include <memory>
#include <vector>
#include <array>

namespace vis {
    class ImgStack : public rclcpp::Node {
        public:
        /// @brief Constructor for the Walker class
        /// @param name
        ImgStack(std::string name);
        /// @brief Destructor for the Walker class
        ~ImgStack();
        /// @brief Callback function for the image subscriber
        /// @param msg
        void image_callback(const sensor_msgs::msg::Image& msg);
        void get_img();
        cv::Mat thresh_img(cv::Mat image);
        cv::Mat find_contour(cv::Mat image);
        void get_coords(cv::Mat image, std::string color);
        void block_locator(cv::Mat image);

        private: 
        /// @brief Coordinate and orientation message
        std::vector<std::array<double,4>> coord_msg;
        /// @brief Input image
        cv::Mat input_image;
        panda_world::msg::MultiBlockLocation  block_locations;
        /// @brief Subscriber
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
        /// @brief Publisher
        // rclcpp::Publisher<coord_msg>::SharedPtr publisher_; //check the argument
        // std::deque<std::pair<std::string, geometry_msgs::msg::Pose2D>>   block_locations;

    };
};
#endif // PANDA_CONTROLLER_INCLUDE_IMG_STACK_HPP_double