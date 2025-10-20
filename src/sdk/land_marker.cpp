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

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <angles/angles.h>

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
        // rotation to camera
        node_handle_->declare_parameter<std::vector<double>>("rotation_to_camera", std::vector<double>());
        std::vector<double> rotationToCam = node_handle_->get_parameter("rotation_to_camera").as_double_array();
        if (rotationToCam.size() < 3)
        {
            rotationToCam.resize(3, 0.0);
        }
        rotation_to_.transform.translation.x = 0.0;
        rotation_to_.transform.translation.y = 0.0;
        rotation_to_.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(angles::from_degrees(rotationToCam[0]), angles::from_degrees(rotationToCam[1]),
            angles::from_degrees(rotationToCam[2]));
        rotation_to_.transform.rotation = tf2::toMsg(q);
        // transform to
        node_handle_->declare_parameter<std::vector<double>>("transform_to", std::vector<double>());
        std::vector<double> transformTo = node_handle_->get_parameter("transform_to").as_double_array();
        if (transformTo.size() < 6)
        {
            transformTo.resize(6, 0.0);
        }
        transform_to_.transform.translation.x = transformTo[0];
        transform_to_.transform.translation.y = transformTo[1];
        transform_to_.transform.translation.z = transformTo[2];
        q.setRPY(angles::from_degrees(transformTo[3]), angles::from_degrees(transformTo[4]),
            angles::from_degrees(transformTo[5]));
        transform_to_.transform.rotation = tf2::toMsg(q);

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

    static geometry_msgs::msg::Pose applyTransform(const geometry_msgs::msg::Pose& Src,
        const geometry_msgs::msg::TransformStamped& Transform)
    {
        // apply the transform to the pose
        geometry_msgs::msg::Pose transformed;
        tf2::doTransform(Src, transformed, Transform);

        return transformed;
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
                if (rclcpp::spin_until_future_complete(node_client_handle_, result, std::chrono::seconds(5)) == rclcpp::FutureReturnCode::SUCCESS)
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
            msg.pose.covariance.fill(0.0);

            RCLCPP_INFO_STREAM(node_handle_->get_logger(), "\033[1;32m" <<
                "sucessfully get the transform from QR code to camera, translation: [" << offset.pose.position.x <<
                ", " << offset.pose.position.y << ", " << offset.pose.position.z << "], eulers[" << eulers[0] <<
                ", " << eulers[1] << ", " << eulers[2] << "]" << "\033[0m");

            // align to camera first
            msg.pose.pose = applyTransform(offset.pose, rotation_to_);
            tf2::Quaternion qAligned(msg.pose.pose.orientation.x,
                 msg.pose.pose.orientation.y,
                 msg.pose.pose.orientation.z,
                 msg.pose.pose.orientation.w);
            msg.pose.pose.orientation = tf2::toMsg(qAligned.inverse());

            // convert to eulers
            tf2::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
            std::vector<double> eulersRad;
            eulersRad.resize(3, 0.0);
  		    tf2::Matrix3x3(q).getRPY(eulersRad[0], eulersRad[1], eulersRad[2]);

            RCLCPP_INFO_STREAM(node_handle_->get_logger(), "aligned to camera, translation: [" <<
                msg.pose.pose.position.x << ", " << msg.pose.pose.position.y << ", " << msg.pose.pose.position.z <<
                "], eulers[" << angles::to_degrees(eulersRad[0]) << ", " << angles::to_degrees(eulersRad[1]) <<
                ", " << angles::to_degrees(eulersRad[2]) << "]");

            // transform to user specified
            auto transformed = applyTransform(msg.pose.pose, transform_to_);
            msg.pose.pose = transformed;

            RCLCPP_INFO_STREAM(node_handle_->get_logger(), "transformed to " << msg.header.frame_id << ", translation: [" <<
                msg.pose.pose.position.x << ", " << msg.pose.pose.position.y << ", " << msg.pose.pose.position.z <<
                "], eulers[" << angles::to_degrees(eulersRad[0]) << ", " << angles::to_degrees(eulersRad[1]) <<
                ", " << angles::to_degrees(eulersRad[2]) << "]");

            // publish
            pub_rtab_landmark_->publish(msg);

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
