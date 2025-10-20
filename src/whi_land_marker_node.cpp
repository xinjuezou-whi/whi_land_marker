/******************************************************************
node of landmark

Features:
- send landmark message while marker is detected
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2025-10-17: Initial version
2025-xx-xx: xxx
******************************************************************/
#include "whi_land_marker/land_marker.h"

#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <signal.h>
#include <functional>

#define ASYNC 1

// since ctrl-c break cannot trigger descontructor, override the signal interruption
std::function<void(int)> functionWrapper;
void signalHandler(int Signal)
{
	functionWrapper(Signal);
}

int main(int argc, char** argv)
{
	/// node version and copyright announcement
	std::cout << "\nWHI land markder VERSION 00.00.3" << std::endl;
	std::cout << "Copyright © 2025-2026 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

	/// ros infrastructure
    rclcpp::init(argc, argv);

    // create node
    const std::string nodeName("whi_land_markder"); 
    auto nodeHandle = std::make_shared<rclcpp::Node>(nodeName);

	/// node logic
	auto instance = std::make_unique<whi_land_marker::LandMarker>(nodeHandle);

	// override the default ros sigint handler, with this override the shutdown will be gracefull
    // NOTE: this must be set after the NodeHandle is created
	signal(SIGINT, signalHandler);
	functionWrapper = [&](int)
	{
		instance.reset(nullptr);

		// all the default sigint handler does is call shutdown()
        if (rclcpp::ok())
        {
            rclcpp::shutdown();
        }
	};

	/// ros spinner
	// NOTE: We run the ROS loop in a separate thread as external calls such as
	// service callbacks to load controllers can block the (main) control loop
#if ASYNC
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(nodeHandle);
    executor->spin();  // blocking until shutdown
#else
    rclcpp::spin(nodeHandle);
#endif

	std::cout << nodeName << " exited" << std::endl;

	return 0;
}
