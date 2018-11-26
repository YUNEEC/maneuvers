//
// Simple example to demonstrate how to use the Dronecode SDK.
//
// Author: Julian Oes <julian@oes.ch>

#include <chrono>
#include <cstdint>
#include <system.h>
#include <plugins/action/action.h>
#include <dronecode_sdk.h>
#include <plugins/telemetry/telemetry.h>
#include <plugins/log_files/log_files.h>
#include <iostream>
#include <thread>
#include <math.h>

using namespace dronecode_sdk;
using namespace std::this_thread;
using namespace std::chrono;

#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour

void usage(std::string bin_name)
{
    std::cout << NORMAL_CONSOLE_TEXT << "Usage : " << bin_name << " <connection_url>" << std::endl
              << "Connection URL format should be :" << std::endl
              << " For TCP : tcp://[server_host][:server_port]" << std::endl
              << " For UDP : udp://[bind_host][:bind_port]" << std::endl
              << " For Serial : serial:///path/to/serial/dev[:baudrate]" << std::endl
              << "For example, to connect to the simulator use URL: udp://:14540" << std::endl;
}



void component_discovered(ComponentType component_type)
{
    std::cout << NORMAL_CONSOLE_TEXT << "Discovered a component with type "
              << unsigned(component_type) << std::endl;
}



int detect_system(int argc, char **argv, DronecodeSDK &dc)
{
    std::string connection_url;
    ConnectionResult connection_result;

    if (argc == 2) {
        connection_url = argv[1];
        connection_result = dc.add_any_connection(connection_url);
    } else {
        usage(argv[0]);
        return 1;
    }

    if (connection_result != ConnectionResult::SUCCESS) {
        std::cout << ERROR_CONSOLE_TEXT
                  << "Connection failed: " << connection_result_str(connection_result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

    return 0;
}



int system_setup(DronecodeSDK &dc, System &system)
{
    bool discovered_system = false;

    // We don't need to specify the UUID if it's only one system anyway.
    // If there were multiple, we could specify it with:
    // dc.system(uint64_t uuid);

    std::cout << "Waiting to discover system..." << std::endl;
    dc.register_on_discover([&discovered_system](uint64_t uuid) {
        std::cout << "Discovered system with UUID: " << uuid << std::endl;
        discovered_system = true;
    });

    // We usually receive heartbeats at 1Hz, therefore we should find a system after around 2
    // seconds.
    sleep_for(seconds(2));

    if (!discovered_system) {
        std::cout << ERROR_CONSOLE_TEXT << "No system found, exiting." << NORMAL_CONSOLE_TEXT
                  << std::endl;
        return 1;
    }

    // Register a callback so we get told when components (camera, gimbal) etc
    // are found.
    system.register_component_discovered_callback(component_discovered);

    return 0;
}



Telemetry::Position calculate_setpoint(double lat_m, double long_m, double height_above_home, Telemetry::Position current_position){
    // calculate new setpoint
    Telemetry::Position new_position;

    new_position.latitude_deg = current_position.latitude_deg + lat_m / 111111;  // because 1 deg lat == 111'111 meters
    new_position.longitude_deg = current_position.longitude_deg + long_m / (111111*cos(new_position.latitude_deg));
    new_position.absolute_altitude_m = current_position.absolute_altitude_m - current_position.relative_altitude_m + height_above_home;

    return new_position;
}



int arm_and_takeoff(Telemetry *telemetry, Action *action)
{
        // Check if vehicle is ready to arm
    while (telemetry->health_all_ok() != true) {
        std::cout << "Vehicle is getting ready to arm" << std::endl;
        sleep_for(seconds(1));
    }

    // Arm vehicle
    std::cout << "Arming..." << std::endl;
    const ActionResult arm_result = action->arm();

    if (arm_result != ActionResult::SUCCESS) {
        std::cout << ERROR_CONSOLE_TEXT << "Arming failed:" << action_result_str(arm_result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

    // Take off
    auto takeoff_altitude_result = action->get_takeoff_altitude().second;
    std::cout << "Taking off to height " << takeoff_altitude_result << " meters" << std::endl;
    const ActionResult takeoff_result = action->takeoff();
    if (takeoff_result != ActionResult::SUCCESS) {
        std::cout << ERROR_CONSOLE_TEXT << "Takeoff failed:" << action_result_str(takeoff_result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

    // wait until drone has reached takeoff height
    while ((action->get_takeoff_altitude().second - 0.2) > telemetry->position().relative_altitude_m) {
        std::cout << TELEMETRY_CONSOLE_TEXT 
        << "Relative height: " << telemetry->position().relative_altitude_m 
        <<  NORMAL_CONSOLE_TEXT<< std::endl;
        sleep_for(seconds(1));
    }
    return 0;
}



int trigger_RTL(Telemetry *telemetry, Action *action)
{
    // Make RTL right over home position
    std::cout << "trigger RTL" << std::endl;
    const ActionResult rtl_result = action->return_to_launch();
    if (rtl_result != ActionResult::SUCCESS) {
        //RTL failed, so exit (in reality might send kill command.)
        return 1;
    }

    // We are relying on auto-disarming but let's keep watching the telemetry until it is disarmed
    while (telemetry->armed()) {
        // show height in terminal
        std::cout << TELEMETRY_CONSOLE_TEXT 
        << "Relative height: " << telemetry->position().relative_altitude_m 
        <<  NORMAL_CONSOLE_TEXT<< std::endl;

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "Disarmed, ready for next part of maneuver." << std::endl;
    
    return 0;
}



int goto_setpoint_and_RTL(Telemetry *telemetry, Action *action, double lat_m, double long_m, double height_above_home, double yaw)
{
    // calculate new position in longitude and latitude and height above sea level
    Telemetry::Position position_setpoint = calculate_setpoint(lat_m, long_m, height_above_home, telemetry->position());

    // take off to start maneuver
    auto return_value = arm_and_takeoff(telemetry, action);
    if (return_value != 0){
        return return_value;
    } 

    // send the drone away from home to a new setpoint
    const ActionResult location_result = action->goto_location(position_setpoint.latitude_deg, position_setpoint.longitude_deg, position_setpoint.absolute_altitude_m, yaw);
    if (location_result != ActionResult::SUCCESS) {
        std::cout << ERROR_CONSOLE_TEXT << "going to new location failed failed:" << action_result_str(location_result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

    // TODO: wait until new setpoint is reached
    int i = 0;
    while (i < 15)
    {
        // show height in terminal
        std::cout << TELEMETRY_CONSOLE_TEXT 
        << "Relative height: " << telemetry->position().relative_altitude_m 
        <<  NORMAL_CONSOLE_TEXT<< std::endl;
        sleep_for(seconds(1));
        i++;
    }
    
    return_value = trigger_RTL(telemetry, action);

    return return_value;
}



int main(int argc, char **argv)
{
    DronecodeSDK dc;

    int return_value = detect_system(argc, argv, dc);
    if (return_value != 0){
        return return_value;
    } 

    System &system = dc.system();

    return_value = system_setup(dc, system);
    if (return_value != 0){
        return return_value;
    } 

    auto telemetry = std::make_shared<Telemetry>(system);
    auto action = std::make_shared<Action>(system);

    // We want to listen to the altitude of the drone at 1 Hz.
    const Telemetry::Result set_rate_result = telemetry->set_rate_position(1.0);
    if (set_rate_result != Telemetry::Result::SUCCESS) {
        std::cout << ERROR_CONSOLE_TEXT
                  << "Setting rate failed:" << Telemetry::result_str(set_rate_result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

    std::cout << "Trigger RTL at takeoff height and directly above home" << std::endl;
    return_value = arm_and_takeoff(telemetry.get(), action.get());
    if (return_value != 0){
        return return_value;
    } 

    // land directly over home position (from takeoff height)
    return_value = trigger_RTL(telemetry.get(), action.get());
    
    if (return_value != 0){
        return return_value;
    } 

    // set a new setpoint away from home
    double lat_m = 3;
    double long_m = 0;
    double height_above_home = 1;
    double yaw = 0;
    std::cout << "Fly less than RTL_CONE_DIST meters away (default: 5m). Drone should not rise up to RTL_RETURN_ALT but only to a height given by a cone" << std::endl;
    return_value = goto_setpoint_and_RTL(telemetry.get(), action.get(), lat_m, long_m, height_above_home, yaw);

    if (return_value != 0){
        return return_value;
    } 

    // set a new setpoint away from home
    lat_m = 6;
    long_m = 0;
    height_above_home = 4;
    yaw = 0;
    std::cout << "Fly away more than RTL_CONE_DIST (drone should rise all the way up to RTL_RETURN_ALT)" << std::endl;
    return_value = goto_setpoint_and_RTL(telemetry.get(), action.get(), lat_m, long_m, height_above_home, yaw);

    if (return_value != 0){
        return return_value;
    } 

    // set a new setpoint away from home
    lat_m = 10;
    long_m = 0;
    height_above_home = 35;
    yaw = 0;
    std::cout << "Fly away more than RTL_CONE_DIST and above RTL_RETURN_ALT" << std::endl;
    return_value = goto_setpoint_and_RTL(telemetry.get(), action.get(), lat_m, long_m, height_above_home, yaw);
    
    if (return_value != 0){
        return return_value;
    } 

    // set a new setpoint away from home
    lat_m = 3;
    long_m = 0;
    height_above_home = 15;
    yaw = 0;
    std::cout << "Fly away less than RTL_CONE_DIST but above the cone" << std::endl;
    return_value = goto_setpoint_and_RTL(telemetry.get(), action.get(), lat_m, long_m, height_above_home, yaw);

    // TODO: Download logfiles after flight (code below stops downloading at 89%...)
    // The log files are also stored here(directly from SITL):  Firmware/build/posix_sitl_default/tmp/rootfs/fs/microsd/log

    // auto log_files = std::make_shared<LogFiles>(system);
    // const std::string file_path = "/home/philipp/.log";
    // unsigned id = 1;
    // std::pair<LogFiles::Result, std::vector<LogFiles::Entry> > log_files_entries = log_files->get_entries();
    // auto nr_of_entries = log_files_entries.second.size();
    // auto byte_size = log_files_entries.second[1].size_bytes;
    // std::cout << "Nr of Log files: " << nr_of_entries << std::endl;
    // std::cout << "byte size of file 1 :" << byte_size << std::endl;
    // LogFiles::Result Log_result = log_files->download_log_file(id, file_path);

    return return_value;
}