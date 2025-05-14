
#include "rune_solver/rune_solver_node.hpp"

namespace imca::rune
{
  RuneSolverNode::RuneSolverNode(const rclcpp::NodeOptions &options) : Node("rune_solver", options)
  {
    IMCA_REGISTER_LOGGER("rune_solver", "~/fyt2024-log", INFO);
    IMCA_INFO("rune_solver", "Starting RuneSolverNode!");
    // Enable/Disable Rune Solver
    set_mode_srv_ = this->create_service<rm_interfaces::srv::SetMode>(
        "rune_solver/set_mode",
        std::bind(
            &RuneSolverNode::setModeCallback, this, std::placeholders::_1, std::placeholders::_2));
    // Tf2 info
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    // RuneSolver
    auto rune_solver_params = RuneSolver::RuneSolverParams{
        .compensator_type = declare_parameter("compensator_type", "ideal"),
        .rune_mode = static_cast<RuneMode>(declare_parameter("mode", 0)),
        .gravity = declare_parameter("gravity", 9.8),
        .bullet_speed = declare_parameter("bullet_speet", 28.0),
        .MIN_DISTANCE_TO_TARGET = declare_parameter("min_distance_to_target", 2.0),
        .MAX_DISTANCE_TO_TARGET = declare_parameter("max_distance_to_target", 10.0),
        .MIN_FIT_DATA_SIZE = static_cast<int>(declare_parameter("min_dit_data_size", 20)),
        .MAX_FIT_DATA_SIZE = static_cast<int>(declare_parameter("max_dit_data_size", 1200)),
        .COMPANSATE_TIME = declare_parameter("predict_time", 0.0),
        .COMPANSATE_PITCH = declare_parameter("compansate_pitch", 0.0),
        .COMPANSATE_YAW = declare_parameter("compansate_yaw", 0.0)};

    rune_solver_ = std::make_unique<RuneSolver>(rune_solver_params, tf2_buffer_);
    // Target subscriber
    rune_target_sub_ = this->create_subscription<rm_interfaces::msg::RuneTarget>(
        "rune_detector/rune_target",
        rclcpp::SensorDataQoS(),
        std::bind(&RuneSolverNode::runeTargetCallback, this, std::placeholders::_1));

    // Camera info
    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info",
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info)
        {
          cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
          rune_solver_->pnp_solver = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
          rune_solver_->pnp_solver->setObjectPoints("rune", RUNE_OBJECT_POINTS);
          cam_info_sub_.reset();
        });

    // Publisher
    gimbal_pub_ = this->create_publisher<rm_interfaces::msg::GimbalCmd>("rune_solver/cmd_gimbal",
                                                                        rclcpp::SensorDataQoS());
    target_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "rune_solver/predict_target", rclcpp::SensorDataQoS());
    // Debug info
    debug_ = this->declare_parameter("debug", true);
    mesurement_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
        "rune_solver/mesurement_target", rclcpp::SensorDataQoS());
    if (debug_)
    {
      r_tag_pos_marker_.ns = "r_tag_position";
      r_tag_pos_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      r_tag_pos_marker_.scale.x = r_tag_pos_marker_.scale.y = r_tag_pos_marker_.scale.z = 0.15;
      r_tag_pos_marker_.text = "R";
      r_tag_pos_marker_.color.a = 1.0;
      r_tag_pos_marker_.color.r = 1.0;
      r_tag_pos_marker_.color.g = 1.0;
      obs_pos_marker_.ns = "observed_position";
      obs_pos_marker_.type = visualization_msgs::msg::Marker::SPHERE;
      obs_pos_marker_.scale.x = obs_pos_marker_.scale.y = obs_pos_marker_.scale.z = 0.308;
      obs_pos_marker_.color.a = 1.0;
      obs_pos_marker_.color.g = 1.0;
      pred_pos_marker_.ns = "predicted_position";
      pred_pos_marker_.type = visualization_msgs::msg::Marker::SPHERE;
      pred_pos_marker_.scale.x = pred_pos_marker_.scale.y = pred_pos_marker_.scale.z = 0.308;
      pred_pos_marker_.color.a = 1.0;
      pred_pos_marker_.color.r = 1.0;
      aimming_line_marker_.ns = "aimming_line";
      aimming_line_marker_.type = visualization_msgs::msg::Marker::ARROW;
      aimming_line_marker_.scale.x = 0.03;
      aimming_line_marker_.scale.y = 0.05;
      aimming_line_marker_.color.a = 0.5;
      aimming_line_marker_.color.r = 1.0;
      aimming_line_marker_.color.b = 1.0;
      aimming_line_marker_.color.g = 1.0;
      marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "rune_solver/marker", rclcpp::SensorDataQoS());
      predict_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("rune_solver/predict_point", rclcpp::SensorDataQoS());
    }
  }

  void RuneSolverNode::setModeCallback(const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
                                       std::shared_ptr<rm_interfaces::srv::SetMode::Response> response)
  {
    response->success = true;
    VisionMode mode = static_cast<VisionMode>(request->mode);

    switch (mode)
    {
    case VisionMode::SMALL_RUNE:
    {
      enable_ = true;
      rune_solver_->rune_solver_params.rune_mode = RuneMode::SMALL;
      rune_solver_->stopFitThread();
    }
    case VisionMode::BIG_RUNE:
    {
      enable_ = true;
      rune_solver_->rune_solver_params.rune_mode = RuneMode::BIG;
      rune_solver_->stopFitThread();
      rune_solver_->startFitThread();
      break;
    }
    default:
    {
      rune_solver_->stopFitThread();
      enable_ = false;
      break;
    }
    }
  }

  void RuneSolverNode::runeTargetCallback(
      const rm_interfaces::msg::RuneTarget::SharedPtr rune_target_msg)
  {
    if (!enable_)
      return;
    // rune_solver_->pnp_solver is nullptr when camera_info is not received
    if (rune_solver_->pnp_solver == nullptr)
    {
      return;
    }
    geometry_msgs::msg::PointStamped predict_point_msg, mesurement_point_msg;
    rm_interfaces::msg::GimbalCmd control_msg;
    mesurement_point_msg.header.stamp = rune_target_msg->header.stamp;

    std::vector<cv::Point2f> points;
    visualization_msgs::msg::MarkerArray marker_array;
    for (auto point : rune_target_msg->pts)
    {
      points.emplace_back(cv::Point2f(point.x, point.y));
    }
    if (rune_solver_->calculate(points, rune_target_msg->header.stamp))
    {
      Eigen::Vector3d cur_pose = rune_solver_->getCurPose();
      mesurement_point_msg.point.x = cur_pose.x();
      mesurement_point_msg.point.y = cur_pose.y();
      mesurement_point_msg.point.z = cur_pose.z();
      double flying_time = rune_solver_->trajectory_compensator->getFlyingTime(cur_pose);
      rclcpp::Time predict_timestamp = this->now() + rclcpp::Duration::from_seconds(rune_solver_->rune_solver_params.COMPANSATE_TIME) +
                                       rclcpp::Duration::from_seconds(flying_time);
      predict_point_msg.header.stamp = predict_timestamp;
      Eigen::Vector3d pred_pos = Eigen::Vector3d::Zero();
      double predict_angle = rune_solver_->preidctTarget(pred_pos, predict_timestamp.seconds());
      mesurement_pub_->publish(std::move(mesurement_point_msg));
      if (predict_angle != 0)
      {
        predict_point_msg.point.x = pred_pos.x();
        predict_point_msg.point.y = pred_pos.y();
        predict_point_msg.point.z = pred_pos.z();
        target_pub_->publish(std::move(predict_point_msg));
        if (debug_)
        {
          geometry_msgs::msg::PointStamped point;
          auto predict_pixel=rune_solver_->getPredictPixel();
          point.point.x=predict_pixel.x;
          point.point.y=predict_pixel.y;
          point.header.stamp = rune_target_msg->header.stamp;
          predict_point_pub_->publish(std::move(point));
        }

        try
        {
          control_msg = rune_solver_->solveGimbalCmd(pred_pos);
        }
        catch (...)
        {
          IMCA_ERROR("rune_solver", "solveGimbalCmd error");
          control_msg.yaw_diff = 0;
          control_msg.pitch_diff = 0;
          control_msg.distance = -1;
          control_msg.pitch = 0;
          control_msg.yaw = 0;
          control_msg.fire_advice = false;
        }
      }

      if (debug_)
      {
        obs_pos_marker_.header.frame_id = "odom";
        obs_pos_marker_.header.stamp = rune_target_msg->header.stamp;
        obs_pos_marker_.action = visualization_msgs::msg::Marker::ADD;
        obs_pos_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);
        obs_pos_marker_.pose.position.x = mesurement_point_msg.point.x;
        obs_pos_marker_.pose.position.y = mesurement_point_msg.point.y;
        obs_pos_marker_.pose.position.z = mesurement_point_msg.point.z;

        Eigen::Vector3d r_tag_pos = rune_solver_->getCurCenterPose();
        r_tag_pos_marker_.header.frame_id = "odom";
        r_tag_pos_marker_.header.stamp = rune_target_msg->header.stamp;
        r_tag_pos_marker_.action = visualization_msgs::msg::Marker::ADD;
        r_tag_pos_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);
        r_tag_pos_marker_.pose.position.x = r_tag_pos.x();
        r_tag_pos_marker_.pose.position.y = r_tag_pos.y();
        r_tag_pos_marker_.pose.position.z = r_tag_pos.z();
        marker_array.markers.push_back(obs_pos_marker_);
        marker_array.markers.push_back(r_tag_pos_marker_);

        pred_pos_marker_.header.frame_id = "odom";
        pred_pos_marker_.header.stamp = predict_timestamp;
        pred_pos_marker_.action = visualization_msgs::msg::Marker::ADD;
        pred_pos_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);
        pred_pos_marker_.pose.position.x = pred_pos.x();
        pred_pos_marker_.pose.position.y = pred_pos.y();
        pred_pos_marker_.pose.position.z = pred_pos.z();
        marker_array.markers.push_back(pred_pos_marker_);

        aimming_line_marker_.action = visualization_msgs::msg::Marker::ADD;
        aimming_line_marker_.points.clear();
        aimming_line_marker_.header.frame_id = "odom";
        aimming_line_marker_.header.stamp = this->now();
        aimming_line_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);
        geometry_msgs::msg::Point aimming_line_start, aimming_line_end;
        aimming_line_marker_.points.emplace_back(aimming_line_start);
        aimming_line_end.y = 15 * sin(control_msg.yaw * M_PI / 180);
        aimming_line_end.x = 15 * cos(control_msg.yaw * M_PI / 180);
        aimming_line_end.z = 15 * sin(control_msg.pitch * M_PI / 180);
        aimming_line_marker_.points.emplace_back(aimming_line_end);
        marker_array.markers.push_back(aimming_line_marker_);
      }
    }
    else
    {
      if (debug_)
      {
        obs_pos_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
        pred_pos_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
        r_tag_pos_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
        aimming_line_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(obs_pos_marker_);
        marker_array.markers.push_back(pred_pos_marker_);
        marker_array.markers.push_back(r_tag_pos_marker_);
        marker_array.markers.push_back(aimming_line_marker_);
        marker_pub_->publish(marker_array);
      }
    }
    if (debug_)
      marker_pub_->publish(marker_array);
    gimbal_pub_->publish(std::move(control_msg));
  }

  rcl_interfaces::msg::SetParametersResult RuneSolverNode::onSetParameters(
      std::vector<rclcpp::Parameter> parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto &param : parameters)
    {
      if (param.get_name() == "predict_time")
      {
        rune_solver_->rune_solver_params.COMPANSATE_TIME = param.as_double();
      }
      else if (param.get_name() == "debug")
      {
        debug_ = param.as_bool();
      }
      else if (param.get_name() == "gravity")
      {
        rune_solver_->rune_solver_params.gravity = param.as_double();
      }
      else if (param.get_name() == "bullet_speed")
      {
        rune_solver_->rune_solver_params.bullet_speed = param.as_double();
      }
      else if (param.get_name() == "min_dit_data_size")
      {
        rune_solver_->rune_solver_params.MIN_FIT_DATA_SIZE = param.as_int();
      }
      else if (param.get_name() == "max_dit_data_size")
      {
        rune_solver_->rune_solver_params.MAX_FIT_DATA_SIZE = param.as_int();
      }
      else if (param.get_name() == "min_distance_to_target")
      {
        rune_solver_->rune_solver_params.MIN_DISTANCE_TO_TARGET = param.as_double();
      }
      else if (param.get_name() == "max_distance_to_target")
      {
        rune_solver_->rune_solver_params.MAX_DISTANCE_TO_TARGET = param.as_double();
      }
    }
    return result;
  }

} // namespace imca::rune

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(imca::rune::RuneSolverNode)