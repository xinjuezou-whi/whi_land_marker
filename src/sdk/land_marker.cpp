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

        // publishers
        pub_rtab_landmark_ = node_handle_->create_publisher<rtabmap_msgs::msg::LandmarkDetection>("landmark", 10);

        // service clients
        client_activate_ = node_handle_->create_client<std_srvs::srv::SetBool>("qrcode_activate");
        client_qr_code_ = node_handle_->create_client<whi_interfaces::srv::WhiSrvQrcode>("qrcode_pose");

        // service
        service_ = node_handle_->create_service<std_srvs::srv::SetBool>("detect_marker",
            std::bind(&LandMarker::onServiceDetect, this, std::placeholders::_1, std::placeholders::_2));
    }

    bool LandMarker::onServiceDetect(const std::shared_ptr<std_srvs::srv::SetBool::Request> Request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> Response)
    {
        bool active = false;
        if (client_activate_)
        {
            auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
            request->data = true;
            auto result = client_activate_->async_send_request(request);
            if (rclcpp::spin_until_future_complete(node_handle_, result) == rclcpp::FutureReturnCode::SUCCESS)
            {
                if (result.get()->success)
                {
                    active = true;
                }
            }
            else
            {
                RCLCPP_WARN(node_handle_->get_logger(), "failed to call activate QR code service");
            }
        }

        int id = -1;
        geometry_msgs::msg::PoseStamped offset;
        if (active && client_qr_code_)
        {
            auto request = std::make_shared<whi_interfaces::srv::WhiSrvQrcode::Request>();
            request->count = qr_avg_count_;
            auto result = client_qr_code_->async_send_request(request);
            if (rclcpp::spin_until_future_complete(node_handle_, result) == rclcpp::FutureReturnCode::SUCCESS)
            {
                if (!result.get()->code.empty())
                {
                    id = stoi(result.get()->code);
                    offset = result.get()->offset_pose;
                }
            }
            else
            {
                RCLCPP_WARN(node_handle_->get_logger(), "failed to call QR code service");
            }
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

            return true;
        }
        else
        {
            return false;
        }
    }

} // namespace whi_land_marker
