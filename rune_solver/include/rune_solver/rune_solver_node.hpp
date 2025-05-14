#ifndef RUNE_SOLVER_RUNE_SOLVER_NODE_HPP_
#define RUNE_SOLVER_RUNE_SOLVER_NODE_HPP_

// 3rd party
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
// ros2
#include <message_filters/subscriber.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
// project
#include "rune_solver/rune_solver.hpp"
#include "rune_solver/types.hpp"
#include "rm_interfaces/msg/rune_target.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
#include "rm_interfaces/srv/set_mode.hpp"
#include "rm_utils/common.hpp"
namespace imca::rune
{
    class RuneSolverNode : public rclcpp::Node
    {
    private:
        rclcpp::Service<rm_interfaces::srv::SetMode>::SharedPtr set_mode_srv_;
        bool enable_;
        // Rune solver
        std::unique_ptr<RuneSolver> rune_solver_;
        // Tf message
        std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

        // Camera info part
        std::shared_ptr<sensor_msgs::msg::CameraInfo> cam_info_;
        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
        // Target Subscriber
        rclcpp::Subscription<rm_interfaces::msg::RuneTarget>::SharedPtr rune_target_sub_;
        // GimbalCmd Publisher
        rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_pub_;
        // Predict Point Publisher
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr predict_point_pub_;

        // debug
        bool debug_;
        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr mesurement_pub_;

        void runeTargetCallback(const rm_interfaces::msg::RuneTarget::SharedPtr rune_target_msg);
        void setModeCallback(const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
                             std::shared_ptr<rm_interfaces::srv::SetMode::Response> response);
        // Dynamic Parameter
        rcl_interfaces::msg::SetParametersResult onSetParameters(
            std::vector<rclcpp::Parameter> parameters);
        rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

        visualization_msgs::msg::Marker obs_pos_marker_;
        visualization_msgs::msg::Marker r_tag_pos_marker_;
        visualization_msgs::msg::Marker pred_pos_marker_;
        visualization_msgs::msg::Marker aimming_line_marker_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    public:
        RuneSolverNode(const rclcpp::NodeOptions &options);
    };

} // namespace imca::rune
#endif