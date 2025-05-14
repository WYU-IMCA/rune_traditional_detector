// ros2
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
// project
#include "rune_detector/rune_detector.hpp"
#include "rm_interfaces/msg/rune_target.hpp"
#include "rm_interfaces/srv/set_mode.hpp"
#include "rm_utils/logger/log.hpp"

namespace imca::rune
{
    class RuneDetectorNode : public rclcpp::Node
    {
    public:
        RuneDetectorNode(const rclcpp::NodeOptions &options);

    private:
        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg);
        std::unique_ptr<RuneDetector> initDetector();
        void createDebugPublishers();
        void destroyDebugPublishers();
        void setModeCallback(
            const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
            std::shared_ptr<rm_interfaces::srv::SetMode::Response> response);
        // Target publisher
        std::string frame_id_;
        rclcpp::Publisher<rm_interfaces::msg::RuneTarget>::SharedPtr rune_pub_;
        // Enable/Disable Rune Detector
        rclcpp::Service<rm_interfaces::srv::SetMode>::SharedPtr set_rune_mode_srv_;
        // Dynamic Parameter
        rcl_interfaces::msg::SetParametersResult onSetParameters(
            std::vector<rclcpp::Parameter> parameters);
        rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;
        // Image subscription
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
        // Rune Detector
        std::unique_ptr<RuneDetector> detector_;
        EnemyColor detect_color_;
        bool debug_;
        image_transport::Publisher result_img_pub_;
        image_transport::Publisher image_arrow_pub_;
        image_transport::Publisher image_armor_pub_;
        image_transport::Publisher image_center_pub_;
        bool enable_;
    };
} // namespace imca::rune
