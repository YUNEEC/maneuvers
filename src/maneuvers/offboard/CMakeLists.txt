cmake_minimum_required(VERSION 3.2)

project(maneuvers_q_yr_th)

add_executable(maneuvers_q_yr_th
    q_yr_th.cpp)

set_property(TARGET maneuvers_q_yr_th PROPERTY CXX_STANDARD 11 )
target_compile_options(maneuvers_q_yr_th PUBLIC -pthread PRIVATE -Wno-format-security -Wno-literal-suffix )

# library dependency
target_link_libraries(maneuvers_q_yr_th
    mavsdk
    mavsdk_action
    mavsdk_offboard
    mavsdk_telemetry
    mavsdk_param
    mavsdk_mocap
    pthread
)




