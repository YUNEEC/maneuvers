/**
 * Simple quaternion-yaw_rate-thrust maneuver
 **/

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <unistd.h>
#include <future>

#include "plugins/action/action.h"
#include "plugins/offboard/offboard.h"
#include "plugins/telemetry/telemetry.h"
#include "plugins/mocap/mocap.h"
#include "plugins/param/param.h"
#include "mavsdk.h"


using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

#define ERROR_CONSOLE_TEXT "\033[31m"     // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m"     // Restore normal console colour

std::atomic<bool> STOP_VISION;

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

// Logs during Offboard control
inline void offboard_log(const std::string &offb_mode, const std::string msg)
{
    std::cout << "[" << offb_mode << "] " << msg << std::endl;
}

/**
 * Does Offboard control using attitude commands.
 *
 * returns true if everything went well in Offboard control, exits with a log otherwise.
 */
bool offb_ctrl_attitude(std::shared_ptr<mavsdk::Offboard> offboard, std::shared_ptr<mavsdk::Telemetry> telemetry)
{
    const std::string offb_mode = "ATTITUDE";

    // Send it once before starting offboard, otherwise it will be rejected.
    offboard->set_attitude_yaw_rate({0.0f, 0.0f, 0.0f, 0.0f, 0.0f});

    Offboard::Result offboard_result = offboard->start();
    offboard_error_exit(offboard_result, "Offboard start failed");
    offboard_log(offb_mode, "Offboard started");

    offboard_log(offb_mode, "Turn clock-wise and climb");
    offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
    sleep_for(seconds(10));
//
    //offboard_log(offb_mode, "Turn back anti-clockwise");
    //offboard->set_velocity_body({0.0f, 0.0f, 0.0f, -60.0f});
    //sleep_for(seconds(5));
//
    //offboard_log(offb_mode, "Wait for a bit");
    //offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
    //sleep_for(seconds(2));
//
    //offboard_log(offb_mode, "Fly a circle");
    //offboard->set_velocity_body({5.0f, 0.0f, 0.0f, 30.0f});
    //sleep_for(seconds(15));
//
    //offboard_log(offb_mode, "Wait for a bit");
    //offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
    //sleep_for(seconds(5));
//
    //offboard_log(offb_mode, "Fly a circle sideways");
    //offboard->set_velocity_body({0.0f, -5.0f, 0.0f, 30.0f});
    //sleep_for(seconds(15));
//
    //offboard_log(offb_mode, "Wait for a bit");
    //offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
    //sleep_for(seconds(8));
//
    //offboard_result = offboard->stop();
    //offboard_error_exit(offboard_result, "Offboard stop failed: ");
    //offboard_log(offb_mode, "Offboard stopped");

   //for (int i = 0; i < 20; i++) {
   //     // get current attitude
   //    Telemetry::EulerAngle euler = telemetry->attitude_euler_angle();
   //    Telemetry::Quaternion q = telemetry->attitude_quaternion();
////
   //    std::cout << "euler yaw: " << euler.yaw_deg << std::endl;
   //    offboard->set_quaternion_yaw_rate({q.w, q.x, q.y, q.z, 0.0f, 0.0f});
   //    sleep_for(seconds(1)); // Let yaw settle.
////
   //}

    // Now, stop offboard mode.
    //offboard_result = offboard->stop();
    //offboard_error_exit(offboard_result, "Offboard stop failed: ");
    //offboard_log(offb_mode, "Offboard stopped");

    return true;
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


void send_yaw_vision( std::promise<int> *promise, std::shared_ptr<mavsdk::Mocap> mocap, std::shared_ptr<mavsdk::Telemetry> telem)
{
    while (!STOP_VISION) {
        Telemetry::EulerAngle euler = telem->attitude_euler_angle();
        std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
        //std::cout << "millis " << millis  << std::endl;
        std::array<float, 21> cov{};

        float euler_yaw_rad = (euler.yaw_deg / 180.0 * M_PI);

        mocap->set_vision_position_estimate({(uint64_t)millis,
                                            {0.0f, 0.0f, 0.0f},
                                            {0.0f, 0.f, euler_yaw_rad},
                                            cov,0});
        sleep_for(milliseconds(20));
    }

    promise->set_value(1);
}

int main(int argc, char **argv)
{
    Mavsdk dc;
    std::string connection_url;
    ConnectionResult connection_result;

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
    auto param = std::make_shared<Param>(system);
    auto mocap = std::make_shared<Mocap>(system);


    STOP_VISION = false;
    std::promise<int> prom;
    std::future<int> future_result = prom.get_future();
    std::thread vision_thread(&send_yaw_vision, &prom, mocap, telemetry);

    //while (!telemetry->health_all_ok())
    //{
    //    std::cout << "Waiting for system to be ready" << std::endl;
    //    sleep_for(seconds(3));
    //}
//
    std::cout << "System is ready" << std::endl;

    //Action::Result arm_result = action->arm();
    //action_error_exit(arm_result, "Arming failed");
    //std::cout << "Armed" << std::endl;


    //Action::Result takeoff_result = action->takeoff();
    //action_error_exit(takeoff_result, "Takeoff failed");
    //std::cout << "In Air..." << std::endl;
    //sleep_for(seconds(5));

    //  using attitude control
    bool ret = offb_ctrl_attitude(offboard, telemetry);
    if (ret == false) {
        return EXIT_FAILURE;
    }

    //sleep_for(seconds(20));

    const Action::Result land_result = action->land();
    action_error_exit(land_result, "Landing failed");

    // Check if vehicle is still in air
    while (telemetry->in_air()) {
        std::cout << "Vehicle is landing..." << std::endl;
        sleep_for(seconds(1));
    }
    std::cout << "Landed!" << std::endl;

    // We are relying on auto-disarming but let's keep watching the telemetry for a bit longer.
    sleep_for(seconds(3));
    std::cout << "Finished..." << std::endl;

    STOP_VISION = true;

    future_result.get();
    vision_thread.join();

    return true;
}


