/**
 * @file driver.cpp
 * @author Aneesh Chodisetty (aneesch@umd.edu)
 * @brief A simple publisher and subscriber that controls the turtle bot.
 * @version 0.1
 * @date 2022-12-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <walker.h>

#include <memory>

int main(int argc, char* argv[]) {
  // Initializing the rclcpp
  rclcpp::init(argc, argv);

  // Instantiating and spinning the node
  rclcpp::spin(std::make_shared<Walker>());

  // Shutdown
  rclcpp::shutdown();

  return 0;
}
