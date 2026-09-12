cmake_minimum_required(VERSION 3.16)
project(modular_robot_sim)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch models worlds DESTINATION share/${PROJECT_NAME})
ament_package()

