/******************************************************************
class of land marker

Features:
- implemetation land marker
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_land_marker/land_marker.h"

#include <thread>

namespace whi_land_marker
{
    LandMarker::LandMarker(std::shared_ptr<rclcpp::Node>& NodeHandle)
        : node_handle_(NodeHandle)
    {
        init();
    }

    LandMarker::~LandMarker()
    {
    }

    void LandMarker::init()
    {
        // params
        node_handle_->declare_parameter<int>("qr_avg_count", qr_avg_count_);
        qr_avg_count_ = node_handle_->get_parameter("qr_avg_count").as_int();
        node_handle_->declare_parameter<std::string>("cam_frame_id", cam_frame_id_);
        cam_frame_id_ = node_handle_->get_parameter("cam_frame_id").as_string();
        node_handle_->declare_parameter<double>("wait_durining_ptz_available", wait_during_ptz_service_);
        wait_during_ptz_service_ = node_handle_->get_parameter("wait_durining_ptz_available").as_double();

        // publishers
        pub_rtab_landmark_ = node_handle_->create_publisher<rtabmap_msgs::msg::LandmarkDetection>("landmark", 10);

        // service clients
        rclcpp::NodeOptions options;
        options.use_global_arguments(false); // prevents override by rosargs
        node_client_handle_ = std::make_shared<rclcpp::Node>("clients", options);
        client_activate_ = node_client_handle_->create_client<std_srvs::srv::SetBool>("qrcode_activate");
        client_qr_code_ = node_client_handle_->create_client<whi_interfaces::srv::WhiSrvQrcode>("qrcode_pose");
        node_handle_->declare_parameter<std::string>("ptz_home_service", std::string("/ptz_home"));
        auto servicePtzHome = node_handle_->get_parameter("ptz_home_service").as_string();
        client_ptz_home_ = node_client_handle_->create_client<std_srvs::srv::Trigger>(servicePtzHome);

        // service
        service_ = node_handle_->create_service<std_srvs::srv::Trigger>("detect_marker",
            std::bind(&LandMarker::onServiceDetect, this, std::placeholders::_1, std::placeholders::_2));
    }

    bool LandMarker::onServiceDetect(const std::shared_ptr<std_srvs::srv::Trigger::Request> Request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> Response)
    {
        Response->success = execute();

        return Response->success;
    }

    bool LandMarker::execute()
    {
        // check if the ptz_home service is active
        if (client_ptz_home_)
        {
            if (wait_during_ptz_service_ > 0.0)
            {
                if (!client_ptz_home_->wait_for_service(std::chrono::duration<double>(wait_during_ptz_service_)))
                {
                    RCLCPP_WARN_STREAM(node_handle_->get_logger(), "\033[1;33m" <<
                        "PTZ service is not available, PTZ homing will be bypassed" << "\033[0m");
                    wait_during_ptz_service_ = 0.0; // only once detection for the efficiency
                }
            }

            if (client_ptz_home_->service_is_ready())
            {
                auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

                auto result = client_ptz_home_->async_send_request(request);
                if (rclcpp::spin_until_future_complete(node_client_handle_, result) == rclcpp::FutureReturnCode::SUCCESS)
                {
                    auto response = result.get(); // only once
                    if (!response->success)
                    {
                        RCLCPP_WARN_STREAM(node_handle_->get_logger(), "\033[1;33m" <<
                            "failed to activate QR code detection" << "\033[0m");
                    }
                }
            }
        }
        else
        {
            RCLCPP_WARN_STREAM(node_handle_->get_logger(), "\033[1;33m" <<
                "No home PTZ service" << "\033[0m");
        }

        int id = -1;
        geometry_msgs::msg::PoseStamped offset;
        std::vector<double> eulers;
        if (activateMarkDetection(true) && client_qr_code_)
        {
            auto request = std::make_shared<whi_interfaces::srv::WhiSrvQrcode::Request>();
            request->count = qr_avg_count_;

            auto result = client_qr_code_->async_send_request(request);
            if (rclcpp::spin_until_future_complete(node_client_handle_, result) == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto response = result.get(); // only once
                if (!response->code.empty())
                {
                    id = stoi(response->code);
                    offset = response->offset_pose;
                    eulers = response->eulers_degree;
                }
                else
                {
                    RCLCPP_WARN_STREAM(node_handle_->get_logger(), "\033[1;33m" <<
                        "failed to estimate QR code pose" << "\033[0m");
                }
            }

            activateMarkDetection(false);
        }

        if (id >= 0 && pub_rtab_landmark_)
        {
            rtabmap_msgs::msg::LandmarkDetection msg;
            msg.header.stamp = node_handle_->get_clock()->now();
            msg.header.frame_id = cam_frame_id_;
            msg.id = id;
            msg.landmark_frame_id = std::string();
            msg.pose.pose = offset.pose;
            msg.pose.covariance.fill(0.0);

            pub_rtab_landmark_->publish(msg);

            RCLCPP_INFO_STREAM(node_handle_->get_logger(), "\033[1;32m" <<
                "sucessfully get the transform from QR code to " << cam_frame_id_ << ", translation: [" << msg.pose.pose.position.x <<
                ", " << msg.pose.pose.position.y << ", " << msg.pose.pose.position.z << "], eulers[" << eulers[0] <<
                ", " << eulers[1] << ", " << eulers[2] << "]" << "\033[0m");

            return true;
        }
        else
        {
            return false;
        }
    }

    bool LandMarker::activateMarkDetection(bool Flag)
    {
        bool active = false;
        if (client_activate_)
        {
            auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
            request->data = Flag;

            auto result = client_activate_->async_send_request(request);
            if (rclcpp::spin_until_future_complete(node_client_handle_, result) == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto response = result.get(); // only once
                if (response->success)
                {
                    active = Flag;
                }
                else
                {
                    RCLCPP_WARN_STREAM(node_handle_->get_logger(), "\033[1;33m" <<
                        "failed to activate QR code detection" << "\033[0m");
                }
            }
        }

        return active;
    }
} // namespace whi_land_marker
