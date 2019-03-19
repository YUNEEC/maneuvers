/**
 * Simple mission maneuver
 **/

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <unistd.h>
#include <future>

#include "plugins/action/action.h"
#include "dronecode_sdk.h"
#include "plugins/offboard/offboard.h"
#include "plugins/telemetry/telemetry.h"
#include "plugins/mission/mission.h"

using namespace dronecode_sdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

#define ERROR_CONSOLE_TEXT "\033[31m"     // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m"     // Restore normal console colour

// Handles Action's result
inline void action_error_exit(Action::Result result, const std::string &message)
{
    if (result != Action::Result::SUCCESS)
    {
        std::cerr << ERROR_CONSOLE_TEXT << message << Action::result_str(result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Handles Offboard's result
inline void offboard_error_exit(Offboard::Result result, const std::string &message)
{
    if (result != Offboard::Result::SUCCESS)
    {
        std::cerr << ERROR_CONSOLE_TEXT << message << Offboard::result_str(result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Handles connection result
inline void connection_error_exit(ConnectionResult result, const std::string &message)
{
    if (result != ConnectionResult::SUCCESS)
    {
        std::cerr << ERROR_CONSOLE_TEXT << message << connection_result_str(result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Handles Mission's result
inline void handle_mission_err_exit(Mission::Result result, const std::string &message)
{
    if (result != Mission::Result::SUCCESS)
    {
        std::cerr << ERROR_CONSOLE_TEXT << message << Mission::result_str(result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        exit(EXIT_FAILURE);
    }
}

// Logs during Offboard control
inline void offboard_log(const std::string &offb_mode, const std::string msg)
{
    std::cout << "[" << offb_mode << "] " << msg << std::endl;
}

Telemetry::Position computeHorizontalLocation(Telemetry::Position pos, double radius, double bearing)
{
    double earthRadius = 6371000;
    double bearingRadius = ((M_PI * bearing) / 180);
    double latitudeRadius = ((M_PI * (pos.latitude_deg)) / 180);
    double longitudeRadius = ((M_PI * (pos.longitude_deg)) / 180);
    double computedLatitude = asin(sin(latitudeRadius) * cos(radius / earthRadius) + cos(latitudeRadius) * sin(radius / earthRadius) * cos(bearingRadius));
    double computedLongitude = longitudeRadius + atan2(sin(bearingRadius) * sin(radius / earthRadius) * cos(latitudeRadius),
                                                       cos(radius / earthRadius) - sin(latitudeRadius) * sin(computedLatitude));
    Telemetry::Position computepos = {((computedLatitude) * (180.0 / M_PI)), ((computedLongitude) * (180.0 / M_PI)), pos.absolute_altitude_m, pos.relative_altitude_m};
    return computepos;
}

// convenience functin for missiont item
static std::shared_ptr<MissionItem> make_mission_item(double latitude_deg,
                                                      double longitude_deg,
                                                      float relative_altitude_m,
                                                      float speed_m_s,
                                                      bool is_fly_through,
                                                      float gimbal_pitch_deg,
                                                      float gimbal_yaw_deg,
                                                      float loiter_time_s,
                                                      MissionItem::CameraAction camera_action)
{
    std::shared_ptr<MissionItem> new_item(new MissionItem());
    new_item->set_position(latitude_deg, longitude_deg);
    new_item->set_relative_altitude(relative_altitude_m);
    new_item->set_speed(speed_m_s);
    new_item->set_fly_through(is_fly_through);
    new_item->set_gimbal_pitch_and_yaw(gimbal_pitch_deg, gimbal_yaw_deg);
    new_item->set_loiter_time(loiter_time_s);
    new_item->set_camera_action(camera_action);
    return new_item;
}

void usage(std::string bin_name)
{
    std::cout << NORMAL_CONSOLE_TEXT << "Usage : " << bin_name << " <connection_url>" << std::endl
              << "Connection URL format should be :" << std::endl
              << " For TCP : tcp://[server_host][:server_port]" << std::endl
              << " For UDP : udp://[bind_host][:bind_port]" << std::endl
              << " For Serial : serial:///path/to/serial/dev[:baudrate]" << std::endl
              << "For example, to connect to the simulator use URL: udp://:14540" << std::endl;
}

int main(int argc, char **argv)
{
    DronecodeSDK dc;
    std::string connection_url;
    ConnectionResult connection_result;

    auto prom = std::make_shared<std::promise<void>>();
    auto future_result = prom->get_future();

    if (argc == 2)
    {
        connection_url = argv[1];
        connection_result = dc.add_any_connection(connection_url);
    }
    else
    {
        usage(argv[0]);
        return 1;
    }

    if (connection_result != ConnectionResult::SUCCESS)
    {
        std::cout << ERROR_CONSOLE_TEXT
                  << "Connection failed: " << connection_result_str(connection_result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

    // Wait for the system to connect via heartbeat
    while (!dc.is_connected())
    {
        std::cout << "Wait for system to connect via heartbeat" << std::endl;
        sleep_for(seconds(1));
    }

    // System got discovered.
    System &system = dc.system();
    auto action = std::make_shared<Action>(system);
    auto offboard = std::make_shared<Offboard>(system);
    auto telemetry = std::make_shared<Telemetry>(system);
    auto mission = std::make_shared<Mission>(system);

    while (!telemetry->health_all_ok())
    {
        std::cout << "Waiting for system to be ready" << std::endl;
        sleep_for(seconds(3));
    }

    std::cout << "System is ready" << std::endl;
    std::cout << "Creating and uploading mission" << std::endl;

    // get current position
    Telemetry::Position pos = {telemetry->position().latitude_deg, telemetry->position().longitude_deg, telemetry->position().absolute_altitude_m, telemetry->position().relative_altitude_m};

    std::vector<std::shared_ptr<MissionItem>> mission_items;
    mission_items.push_back(make_mission_item(pos.latitude_deg,
                                              pos.longitude_deg,
                                              10.0f,
                                              2.0f,
                                              true,
                                              -60.f,
                                              -90.f,
                                              0.0f,
                                              MissionItem::CameraAction::START_PHOTO_INTERVAL));

    Telemetry::Position next = computeHorizontalLocation(pos, 20, 270);
    mission_items.push_back(make_mission_item(next.latitude_deg,
                                              next.longitude_deg,
                                              10.0f,
                                              2.0f,
                                              true,
                                              -60.f,
                                              -70.0f,
                                              0.0f,
                                              MissionItem::CameraAction::START_PHOTO_INTERVAL));

    next = computeHorizontalLocation(pos, 30, 180);
    mission_items.push_back(make_mission_item(next.latitude_deg,
                                              next.longitude_deg,
                                              10.0f,
                                              2.0f,
                                              true,
                                              -60.f,
                                              -90.0f,
                                              0.0f,
                                              MissionItem::CameraAction::START_PHOTO_INTERVAL));

    next = computeHorizontalLocation(pos, 10, 90);
    mission_items.push_back(make_mission_item(next.latitude_deg,
                                              next.longitude_deg,
                                              10.0f,
                                              2.0f,
                                              true,
                                              -60.f,
                                              -20.0f,
                                              0.0f,
                                              MissionItem::CameraAction::START_PHOTO_INTERVAL));

    {
        std::cout << "Uploading mission..." << std::endl;
        // We only have the upload_mission function asynchronous for now, so we wrap it using
        // std::future.
        auto prom = std::make_shared<std::promise<Mission::Result>>();
        auto future_result = prom->get_future();
        mission->upload_mission_async(mission_items,
                                      [prom](Mission::Result result) { prom->set_value(result); });

        const Mission::Result result = future_result.get();

        if (result != Mission::Result::SUCCESS)
        {
            std::cout << "Mission upload failed (" << Mission::result_str(result) << "), exiting."
                      << std::endl;
            return 1;
        }

        std::cout << "Mission uploaded." << std::endl;
    }

    // We want to listen to the local posiiton of the drone at 1 Hz.
    const Telemetry::Result set_rate_result = telemetry->set_rate_position_velocity_ned(1.0);

    if (set_rate_result != Telemetry::Result::SUCCESS)
    {
        std::cout << ERROR_CONSOLE_TEXT
                  << "Setting rate failed:" << Telemetry::result_str(set_rate_result)
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    };

    // Arm
    Action::Result arm_result = action->arm();

    action_error_exit(arm_result, "Arming failed");

    std::cout << "Armed" << std::endl;

    std::atomic<bool> want_to_pause{false};

    // Before starting the mission, we want to be sure to subscribe to the mission progress.
    mission->subscribe_progress([&want_to_pause](int current, int total) {
        std::cout << "Mission status update: " << current << " / " << total << std::endl;
    });

    {
        auto prom = std::make_shared<std::promise<Mission::Result>>();
        auto future_result = prom->get_future();
        mission->start_mission_async([prom](Mission::Result result) {
            prom->set_value(result);
        });

        const Mission::Result result = future_result.get();
        handle_mission_err_exit(result, "Mission start failed: ");
    }

    while (!mission->mission_finished())
    {
        sleep_for(seconds(1));
    }

    {
        // We are done, and can do RTL to go home.
        std::cout << "Commanding RTL" << std::endl;
        const Action::Result result = action->return_to_launch();

        if (result != Action::Result::SUCCESS)
        {
            std::cout << "Failed to command RTL (" << Action::result_str(result) << ")" << std::endl;
        }
    }

    return EXIT_SUCCESS;
}
