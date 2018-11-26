##########
# mission
##########
add_executable(mission
    mission.cpp)
   
set_property(TARGET mission PROPERTY CXX_STANDARD 11)
target_compile_options(mission PRIVATE -Wno-format-security -Wno-literal-suffix)

# library dependency
target_link_libraries(mission
    dronecode_sdk
    dronecode_sdk_action
    dronecode_sdk_offboard
    dronecode_sdk_telemetry
    dronecode_sdk_mission
   )

##########
# RTL_testing
##########

add_executable(RTL_testing
    RTL_testing.cpp)

include_directories(
    px4maneuvers/src/external/Dronecode_sdk/core
    )

set_property(TARGET RTL_testing PROPERTY CXX_STANDARD 11)
target_compile_options(RTL_testing PRIVATE -Wno-format-security -Wno-literal-suffix)

# library dependency
target_link_libraries(RTL_testing
    dronecode_sdk
    dronecode_sdk_action
    dronecode_sdk_offboard
    dronecode_sdk_telemetry
    dronecode_sdk_mission
    dronecode_sdk_offboard
    dronecode_sdk_log_files
   )




