cmake_minimum_required(VERSION 3.2)

project(maneuvers_mission)

add_executable(maneuvers_mission
    mission.cpp)

set_property(TARGET maneuvers_mission PROPERTY CXX_STANDARD 11)
target_compile_options(maneuvers_mission PRIVATE -Wno-format-security -Wno-literal-suffix)

# library dependency
target_link_libraries(maneuvers_mission
    mavsdk
    mavsdk_action
    mavsdk_offboard
    mavsdk_telemetry
    mavsdk_mission
    mavsdk_param
)




