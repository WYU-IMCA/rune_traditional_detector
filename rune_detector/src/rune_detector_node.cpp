// ros2
#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>

#include "rune_detector/rune_detector_node.hpp"

namespace imca::rune
{
    RuneDetectorNode::RuneDetectorNode(const rclcpp::NodeOptions &options) : Node("rune_detector", options)
    {

        IMCA_REGISTER_LOGGER("rune_detector", "~/fyt2024-log", INFO);
        IMCA_INFO("rune_detector", "Starting RuneDetectorNode!");

        // Rune Publisher
        rune_pub_ = this->create_publisher<rm_interfaces::msg::RuneTarget>("rune_detector/rune_target",
                                                                           rclcpp::SensorDataQoS());
        // Debug Publishers
        this->debug_ = declare_parameter("debug", true);
        if (this->debug_)
        {
            createDebugPublishers();
        }
        auto qos = rclcpp::SensorDataQoS();
        qos.keep_last(1);
        img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", qos, std::bind(&RuneDetectorNode::imageCallback, this, std::placeholders::_1));
        set_rune_mode_srv_ = this->create_service<rm_interfaces::srv::SetMode>(
            "rune_detector/set_mode",
            std::bind(
                &RuneDetectorNode::setModeCallback, this, std::placeholders::_1, std::placeholders::_2));
        detector_ = initDetector();
        IMCA_INFO("rune_detector", "RuneDetectorNode finished!");
    }

    void RuneDetectorNode::createDebugPublishers()
    {
        result_img_pub_ = image_transport::create_publisher(this, "rune_detector/result_img");
        image_arrow_pub_ = image_transport::create_publisher(this, "rune_detector/bin_arrow_img");
        image_armor_pub_ = image_transport::create_publisher(this, "rune_detector/bin_armor_img");
        image_center_pub_ = image_transport::create_publisher(this, "rune_detector/bin_center_img");
    }

    void RuneDetectorNode::destroyDebugPublishers()
    {
        result_img_pub_.shutdown();
        image_arrow_pub_.shutdown();
        image_armor_pub_.shutdown();
        image_center_pub_.shutdown();
    }
    void RuneDetectorNode::setModeCallback(
        const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
        std::shared_ptr<rm_interfaces::srv::SetMode::Response> response)
    {
        response->success = true;

        VisionMode mode = static_cast<VisionMode>(request->mode);
        std::string mode_name = visionModeToString(mode);
        if (mode_name == "UNKNOWN")
        {
            IMCA_ERROR("rune_detector", "Invalid mode: {}", request->mode);
            return;
        }

        auto createImageSub = [this]()
        {
            if (img_sub_ == nullptr)
            {
                img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                    "image_raw",
                    rclcpp::SensorDataQoS(),
                    std::bind(&RuneDetectorNode::imageCallback, this, std::placeholders::_1));
            }
        };

        switch (mode)
        {
        case VisionMode::SMALL_RUNE:
        {
            enable_ = true;
            createImageSub();
            break;
        }

        case VisionMode::BIG_RUNE:
        {
            enable_ = true;
            createImageSub();
            break;
        }

        default:
        {
            enable_ = false;
            img_sub_.reset();
            break;
        }
        }

        IMCA_WARN("rune_detector", "Set Rune Mode: {}", visionModeToString(mode));
    }

    void RuneDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr img_msg)
    {
        if (enable_ == false)
        {
            return;
        }

        rm_interfaces::msg::RuneTarget rune_target;
        rune_target.header = img_msg->header;
        auto img = cv_bridge::toCvCopy(img_msg, "rgb8")->image;
        auto final_time = this->now();
        if (detector_->detect(img))
        {

            std::vector<cv::Point2f> points = detector_->getCameraPoints();

            for (size_t i = 0; i < points.size(); i++)
            {
                rune_target.pts[i].x = points[i].x;
                rune_target.pts[i].y = points[i].y;
            }
            rune_pub_->publish(std::move(rune_target));

            if (debug_)
            {
                for (size_t i = 0; i < 2; i++)
                {
                    cv::circle(detector_->m_imageShow,points[i],4,WHITE,-1);
                }
                
                image_center_pub_.publish(
                    cv_bridge::CvImage(img_msg->header, "mono8", detector_->m_imageCenter)
                        .toImageMsg());
            }
        }
        // Draw latency
        if (debug_)
        {
            auto latency = (final_time - img_msg->header.stamp).seconds() * 1000;
            std::stringstream latency_ss;
            latency_ss << "Latency: " << std::fixed << std::setprecision(2) << latency << "ms";
            auto latency_s = latency_ss.str();
            cv::putText(
                detector_->m_imageShow, latency_s, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
            image_arrow_pub_.publish(
                cv_bridge::CvImage(img_msg->header, "mono8", detector_->m_imageArrow)
                    .toImageMsg());
            image_armor_pub_.publish(
                cv_bridge::CvImage(img_msg->header, "mono8", detector_->m_imageArmor)
                    .toImageMsg());
            result_img_pub_.publish(
                cv_bridge::CvImage(img_msg->header, "rgb8", detector_->m_imageShow)
                    .toImageMsg());
        }
    }
    std::unique_ptr<RuneDetector> RuneDetectorNode::initDetector()
    {
        rcl_interfaces::msg::ParameterDescriptor param_desc;
        param_desc.integer_range.resize(1);
        param_desc.description = "0-RED, 1-BLUE";
        param_desc.integer_range[0].from_value = 0;
        param_desc.integer_range[0].to_value = 1;
        detect_color_ = static_cast<EnemyColor>(declare_parameter("detect_color", 0, param_desc));
        param_desc.integer_range[0].step = 1;
        param_desc.integer_range[0].from_value = 0;
        param_desc.integer_range[0].to_value = 255;

        RuneDetector::ArrowParams arrow_params = {
            .blue_brightness_threshold = static_cast<int>(declare_parameter("arrow.blue_brightness_threshold", 140, param_desc)),
            .red_brightness_threshold = static_cast<int>(declare_parameter("arrow.red_brightness_threshold", 150, param_desc)),
            .lightline = {
                .area_min = declare_parameter("arrow.lightline.area_min", 200.),
                .area_max = declare_parameter("arrow.lightline.area_max", 400.),
                .aspect_ratio_max = declare_parameter("arrow.lightline.aspect_ratio_max", 3.),
                .num_min = static_cast<int>(declare_parameter("arrow.lightline.num_min", 2)),
                .num_max = static_cast<int>(declare_parameter("arrow.lightline.num_max", 5)),
            },
            .same_area_ratio_max = declare_parameter("arrow.same_area_ratio_max", 5.),
            .aspect_ratio_min = declare_parameter("arrow.aspect_ratio_min", 2.),
            .aspect_ratio_max = declare_parameter("arrow.aspect_ratio_max", 12.),
            .area_max = declare_parameter("arrow.area_max", 4000.)};

        RuneDetector::ArmorParams armor_params = {
            .blue_brightness_threshold = static_cast<int>(declare_parameter("armor.blue_brightness_threshold", 110, param_desc)),
            .red_brightness_threshold = static_cast<int>(declare_parameter("armor.red_brightness_threshold", 90, param_desc)),
            .armor_contour_area_min = declare_parameter("armor.contour_area_min", 3000.),
            .armor_contour_area_max = declare_parameter("armor.contour_area_max", 5000.),
            .area_ratio_min = declare_parameter("armor.area_ratio_min", 0.8),
            .area_ratio_max = declare_parameter("armor.area_ratio_max", 1.2),
            .armor_center_vertical_distance_threshold = static_cast<int>(declare_parameter("armor.center_vertical_distance_threshold", 90))};

        RuneDetector::centerRParams centerR_params = {
            .area_min = declare_parameter("centerR.area_min", 100.),
            .area_max = declare_parameter("centerR.area_max", 1000.),
            .aspect_ratio_max = declare_parameter("centerR.aspect_ratio_max", 2.)};

        RuneDetector::LocalRoiParams local_roi_params = {
            .distance_ratio = declare_parameter("local_roi_params.distance_ratio", 1.2),
            .width = static_cast<float>(declare_parameter("local_roi_params.width", 200.))};

        // Set dynamic parameter callback
        rcl_interfaces::msg::SetParametersResult onSetParameters(
            std::vector<rclcpp::Parameter> parameters);
        on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&RuneDetectorNode::onSetParameters, this, std::placeholders::_1));

        auto rune_detector = std::make_unique<RuneDetector>(arrow_params, armor_params, centerR_params, local_roi_params, detect_color_, this->debug_);

        return rune_detector;
    }

    rcl_interfaces::msg::SetParametersResult RuneDetectorNode::onSetParameters(
        std::vector<rclcpp::Parameter> parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        for (const auto &param : parameters)
        {
            if (param.get_name() == "detect_color")
            {
                detector_->color = static_cast<EnemyColor>(param.as_int());
            }
            // ArmorParams 参数处理
            else if (param.get_name() == "armor.blue_brightness_threshold")
            {
                detector_->armor_params.blue_brightness_threshold = param.as_int();
            }
            else if (param.get_name() == "armor.red_brightness_threshold")
            {
                detector_->armor_params.red_brightness_threshold = param.as_int();
            }
            else if (param.get_name() == "armor.contour_area_min")
            {
                detector_->armor_params.armor_contour_area_min = param.as_double();
            }
            else if (param.get_name() == "armor.contour_area_max")
            {
                detector_->armor_params.armor_contour_area_max = param.as_double();
            }
            else if (param.get_name() == "armor.area_ratio_min")
            {
                detector_->armor_params.area_ratio_min = param.as_double();
            }
            else if (param.get_name() == "armor.area_ratio_max")
            {
                detector_->armor_params.area_ratio_max = param.as_double();
            }

            // ArrowParams 参数处理
            else if (param.get_name() == "arrow.blue_brightness_threshold")
            {
                detector_->arrow_params.blue_brightness_threshold = param.as_int();
            }
            else if (param.get_name() == "arrow.red_brightness_threshold")
            {
                detector_->arrow_params.red_brightness_threshold = param.as_int();
            }
            else if (param.get_name() == "arrow.lightline.area_min")
            {
                detector_->arrow_params.lightline.area_min = param.as_double();
            }
            else if (param.get_name() == "arrow.lightline.area_max")
            {
                detector_->arrow_params.lightline.area_max = param.as_double();
            }
            else if (param.get_name() == "arrow.lightline.aspect_ratio_max")
            {
                detector_->arrow_params.lightline.aspect_ratio_max = param.as_double();
            }
            else if (param.get_name() == "arrow.lightline.num_min")
            {
                detector_->arrow_params.lightline.num_min = param.as_int();
            }
            else if (param.get_name() == "arrow.lightline.num_max")
            {
                detector_->arrow_params.lightline.num_max = param.as_int();
            }
            else if (param.get_name() == "arrow.same_area_ratio_max")
            {
                detector_->arrow_params.same_area_ratio_max = param.as_double();
            }
            else if (param.get_name() == "arrow.aspect_ratio_min")
            {
                detector_->arrow_params.aspect_ratio_min = param.as_double();
            }
            else if (param.get_name() == "arrow.aspect_ratio_max")
            {
                detector_->arrow_params.aspect_ratio_max = param.as_double();
            }
            else if (param.get_name() == "arrow.area_max")
            {
                detector_->arrow_params.area_max = param.as_double();
            }

            // centerRParams 参数处理
            else if (param.get_name() == "centerR.area_min")
            {
                detector_->centerR_params.area_min = param.as_double();
            }
            else if (param.get_name() == "centerR.area_max")
            {
                detector_->centerR_params.area_max = param.as_double();
            }
            else if (param.get_name() == "centerR.aspect_ratio_max")
            {
                detector_->centerR_params.aspect_ratio_max = param.as_double();
            }

            // LocalRoiParams 参数处理
            else if (param.get_name() == "local_roi_params.distance_ratio")
            {
                detector_->local_roi_params.distance_ratio = param.as_double();
            }
            else if (param.get_name() == "local_roi_params.width")
            {
                detector_->local_roi_params.width = param.as_double();
            }
        }
        result.successful = true;
        return result;
    }
} // namespace imca::rune

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(imca::rune::RuneDetectorNode)