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
