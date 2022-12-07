/**
 * @file walker.h
 * @author Aneesh Chodisetty (aneesch@umd.edu)
 * @brief The declaration of the node that subscribes to LaserScan topic and publishers to cmd_vel.
 * @version 0.1
 * @date 2022-12-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef SIMPLE_WALKER_INCLUDE_SIMPLE_WALKER_WALKER_H_
#define SIMPLE_WALKER_INCLUDE_SIMPLE_WALKER_WALKER_H_

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <string>

class Walker : public rclcpp::Node {
 public:
  Walker(const std::string &node_name = "walker");

 private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
      publisher_;  //!< The pointer to the publisher.
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      subscriber_;                          //!< The pointer to the subscriber.
  geometry_msgs::msg::Twist bot_velocity_;  //!< The calculated bot velocity.

  /**
   * @brief The callback funciton that subscribes to the topic and publishes the
   * calculated velocity.
   *
   * @param msg The laser scan from the /scan topic.
   */
  void topic_callback(const sensor_msgs::msg::LaserScan::ConstPtr &msg);

  /**
   * @brief Method to detect the obstacles in front of the turtlebot.
   *
   * @param msg The msg from the /scan topic.
   * @return true if obstacle is detected.
   */
  bool obstacle_detected(const sensor_msgs::msg::LaserScan::ConstPtr &msg);
};  // Walker

#endif  // SIMPLE_WALKER_INCLUDE_SIMPLE_WALKER_WALKER_H_
