cmake_minimum_required(VERSION 3.2)

project(maneuvers_RTL)

add_executable(maneuvers_RTL
    RTL_testing.cpp)

set_property(TARGET maneuvers_RTL PROPERTY CXX_STANDARD 11)
target_compile_options(maneuvers_RTL PRIVATE -Wno-format-security -Wno-literal-suffix)

# library dependency
target_link_libraries(maneuvers_RTL
    mavsdk
    mavsdk_action
    mavsdk_offboard
    mavsdk_telemetry
    mavsdk_mission
    mavsdk_param
)
