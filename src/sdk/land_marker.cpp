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
#include <tf2/LinearMath/Transform.h>
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
        tf2::Quaternion q;
        q.setRPY(angles::from_degrees(transformTo[3]), angles::from_degrees(transformTo[4]),
            angles::from_degrees(transformTo[5]));
        transform_to_.transform.rotation = tf2::toMsg(q);

        // publishers
        node_handle_->declare_parameter<std::string>("landmark_topic", "landmark");
        auto landmarkTopic = node_handle_->get_parameter("landmark_topic").as_string();
        pub_rtab_landmark_ = node_handle_->create_publisher<rtabmap_msgs::msg::LandmarkDetection>(landmarkTopic, 10);

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

    static geometry_msgs::msg::Transform inverseTransform(const geometry_msgs::msg::Transform& Src)
    {
        tf2::Transform tfSrc;
        tf2::fromMsg(Src, tfSrc);
        tf2::Transform tfInversed = tfSrc.inverse();
        return tf2::toMsg(tfInversed);
    }

    static geometry_msgs::msg::Pose applyTransform(const geometry_msgs::msg::Pose& Src,
        const geometry_msgs::msg::TransformStamped& Transform)
    {
        // apply the transform to the pose
        geometry_msgs::msg::Pose transformed;
        tf2::doTransform(Src, transformed, Transform);

        return transformed;
    }

    std::array<double, 3> toEuler(const geometry_msgs::msg::Quaternion& Quaternion)
    {
        tf2::Quaternion q(Quaternion.x, Quaternion.y, Quaternion.z, Quaternion.w);
        std::array<double, 3> eulers;
        tf2::Matrix3x3(q).getRPY(eulers[0], eulers[1], eulers[2]);

        return eulers;
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
                if (rclcpp::spin_until_future_complete(node_client_handle_, result,
                    std::chrono::duration<double>(wait_during_ptz_service_)) == rclcpp::FutureReturnCode::SUCCESS)
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
        geometry_msgs::msg::Pose offset;
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
                    offset = response->offset_pose.pose;
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
                "sucessfully get the pose of QR code in camera's frame, translation: [" << offset.position.x <<
                ", " << offset.position.y << ", " << offset.position.z << "], eulers[" << eulers[0] <<
                ", " << eulers[1] << ", " << eulers[2] << "]" << "\033[0m");

            // transform to user specified
            auto transformed = applyTransform(offset, transform_to_);
            msg.pose.pose = transformed;

            // convert to eulers
            auto transformedEulers = toEuler(msg.pose.pose.orientation);

            RCLCPP_INFO_STREAM(node_handle_->get_logger(), "the pose of QR code transformed to frame: " << msg.header.frame_id << ", translation: [" <<
                msg.pose.pose.position.x << ", " << msg.pose.pose.position.y << ", " << msg.pose.pose.position.z <<
                "], eulers[" << angles::to_degrees(transformedEulers[0]) << ", " << angles::to_degrees(transformedEulers[1]) <<
                ", " << angles::to_degrees(transformedEulers[2]) << "]");

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
