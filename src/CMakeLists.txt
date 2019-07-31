# At LEAST 2.8 but newer is better
cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
project(PX4maneuvers VERSION 0.1 LANGUAGES CXX)

add_subdirectory(maneuvers)
add_subdirectory(external/MAVSDK)

