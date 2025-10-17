/******************************************************************
class of land marker

Features:
- implemetation land marker
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2025-10-17: Initial version
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include "whi_interfaces/srv/whi_srv_qrcode.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <rtabmap_msgs/msg/landmark_detection.hpp>

namespace whi_land_marker
{
	class LandMarker
	{
    public:
        enum Hardware { HARDWARE_I2C = 0, HARDWARE_SERIAL, HARDWARE_SUM };
        static constexpr const char* hardware[HARDWARE_SUM] = { "i2c", "serial" };

    public:
        LandMarker() = delete;
        LandMarker(std::shared_ptr<rclcpp::Node>& NodeHandle);
        ~LandMarker();

    protected:
        void init();
        bool onServiceDetect(const std::shared_ptr<std_srvs::srv::SetBool::Request> Request,
	        std::shared_ptr<std_srvs::srv::SetBool::Response> Response);

    protected:
        std::shared_ptr<rclcpp::Node> node_handle_{ nullptr };
        // publisher
        rclcpp::Publisher<rtabmap_msgs::msg::LandmarkDetection>::SharedPtr pub_rtab_landmark_{ nullptr };
        // service client
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client_activate_{ nullptr };
        rclcpp::Client<whi_interfaces::srv::WhiSrvQrcode>::SharedPtr client_qr_code_{ nullptr };
        // service
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_{ nullptr };

        // params
        int qr_avg_count_{ 5 };
        std::string cam_frame_id_{ "cam" };
	};
} // namespace whi_land_marker
