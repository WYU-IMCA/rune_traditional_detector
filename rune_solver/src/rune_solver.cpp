#include "rune_solver/rune_solver.hpp"
// third party
#include <angles/angles.h>
namespace imca::rune
{
    RuneSolver::RuneSolver(RuneSolverParams params, std::shared_ptr<tf2_ros::Buffer> tf2_buffer)
        : rune_solver_params(params),
          tf2_buffer_(tf2_buffer),
          m_direction{Direction::UNKNOWN},
          m_convexity{Convexity::UNKNOWN},
          m_totalShift{0},
          m_firstDetect{true},
          m_angleLast{0},
          m_directionThresh{100 / (1000 / FPS) < 2 ? 2 : 100 / (1000 / FPS)}
    {
        trajectory_compensator = CompensatorFactory::createCompensator(params.compensator_type);
        trajectory_compensator->gravity = params.gravity;
        trajectory_compensator->velocity = params.bullet_speed;
        trajectory_compensator->resistance = 0.01;
        m_fitData.reserve(FPS * 20);
        m_fitThread = std::thread(&RuneSolver::fit, this);
        STOP_THREAD.store(false);
        VALID_PARAMS.store(false);
    }

    RuneSolver::~RuneSolver()
    {
        STOP_THREAD.store(true);
        m_fitThread.join();
    }

    void RuneSolver::preprocess(std::vector<cv::Point2f> &cameraPoints, rclcpp::Time timestamp)
    {
        m_cameraPoints = cameraPoints;
        m_timestamp = timestamp;
    }

    void RuneSolver::setFirstDetect()
    {
        if (m_firstDetect)
        {
            m_firstDetect = false;
            m_rMatW2RBase = m_rMatW2R.clone();
            m_startTime = m_timestamp;
        }
    }

    void RuneSolver::directionCal()
    {
        if (m_direction == Direction::UNKNOWN || m_direction == Direction::STABLE)
        {
            m_directionData.push_back(m_angleRel);
            if ((int)m_directionData.size() >= m_directionThresh)
            {
                // 计算角度差并投票
                int stable = 0, anti = 0, clockwise = 0;
                for (size_t i = 0; i < m_directionData.size() / 2; ++i)
                {
                    auto temp{m_directionData.at(i + m_directionData.size() / 2) - m_directionData.at(i)};
                    if (temp > +1.5e-2)
                    {
                        clockwise++;
                    }
                    else if (temp < -1.5e-2)
                    {
                        anti++;
                    }
                    else
                    {
                        stable++;
                    }
                }
                // 得票数最多的为对应旋转方向
                if (int temp{std::max({stable, clockwise, anti})}; temp == clockwise)
                {
                    m_direction = Direction::CLOCKWISE;
                }
                else if (temp == anti)
                {
                    m_direction = Direction::ANTI_CLOCKWISE;
                }
                else
                {
                    m_direction = Direction::STABLE;
                }
            }
        }
    }

    double RuneSolver::preidctTarget(Eigen::Vector3d &pred_pos, double timestamp)
    {
        double angle;
        if (m_direction == Direction::STABLE)
        {
            angle = 0.0;
        }
        else
        {
            if (rune_solver_params.rune_mode == RuneMode::BIG)
            {
                auto frameTime{
                    (m_timestamp - m_startTime).seconds()};
                auto predictTime = timestamp - m_startTime.seconds();
                if (VALID_PARAMS.load() == false)
                {
                    return 0.;
                }
                angle = getAngleBig(predictTime, m_params) - getAngleBig(frameTime, m_params); // 得到大符旋转角度，这里是相对于最后一次识别，预测的旋转角
            }
            else
            {
                angle = getRotationAngleSmall(m_distance2Target, rune_solver_params.bullet_speed,
                                              SMALL_POWER_RUNE_ROTATION_SPEED, rune_solver_params.COMPANSATE_TIME);
            }
            if (m_direction == Direction::ANTI_CLOCKWISE)
            {
                angle = -angle;
            }
        }
        cv::Mat matrixWorld = (cv::Mat_<double>(4, 1) << POWER_RUNE_RADIUS * std::sin(angle),
                               POWER_RUNE_RADIUS - POWER_RUNE_RADIUS * std::cos(angle), 0.0, 1.0);
        cv::Mat matrixRobot{m_matW2R * matrixWorld};
        pred_pos = {(float)(matrixRobot.at<double>(0, 0)), (float)(matrixRobot.at<double>(1, 0)),
                    (float)(matrixRobot.at<double>(2, 0))};
        m_predictRobot = {(float)(matrixRobot.at<double>(0, 0)), (float)(matrixRobot.at<double>(1, 0)),
                          (float)(matrixRobot.at<double>(2, 0))};
        m_predictPixel = getPixelFromRobot(pnp_solver->getCameraMatrix(), m_predictRobot, m_matW2C, m_matW2R);
        return angle;
    }

    /**
     * @brief 预测
     */
    bool RuneSolver::predict()
    {
        double angle;
        if (m_direction == Direction::STABLE)
        {
            angle = 0.0;
        }
        else
        {
            if (rune_solver_params.rune_mode == RuneMode::BIG)
            {
                auto frameTime{
                    (m_timestamp - m_startTime).seconds() * 1e3};
                if (VALID_PARAMS.load() == false)
                {
                    return false;
                }
                angle = getRotationAngleBig(m_distance2Target, rune_solver_params.bullet_speed, m_params, rune_solver_params.COMPANSATE_TIME,
                                            frameTime);
            }
            else
            {
                angle = getRotationAngleSmall(m_distance2Target, rune_solver_params.bullet_speed,
                                              SMALL_POWER_RUNE_ROTATION_SPEED, rune_solver_params.COMPANSATE_TIME);
            }
            if (m_direction == Direction::ANTI_CLOCKWISE)
            {
                angle = -angle;
            }
        }
        cv::Mat matrixWorld = (cv::Mat_<double>(4, 1) << POWER_RUNE_RADIUS * std::sin(angle),
                               POWER_RUNE_RADIUS - POWER_RUNE_RADIUS * std::cos(angle), 0.0, 1.0);
        cv::Mat matrixRobot{m_matW2R * matrixWorld};
        m_predictRobot = {(float)(matrixRobot.at<double>(0, 0)), (float)(matrixRobot.at<double>(1, 0)),
                          (float)(matrixRobot.at<double>(2, 0))};
        auto [predictPitch, predictYaw] = getPitchYawFromRobotCoor(m_predictRobot, rune_solver_params.bullet_speed);
        m_predictPitch = predictPitch;
        m_predictYaw = predictYaw;
        m_predictPixel = getPixelFromRobot(pnp_solver->getCameraMatrix(), m_predictRobot, m_matW2C, m_matW2R);
        return true;
    }
    cv::Point2f getPixelFromRobot(const cv::Mat &intrinsicMatrix, const cv::Point3f &robot, const cv::Mat &w2c, const cv::Mat &w2r)
    {
        cv::Mat matrixRobotPoint = (cv::Mat_<double>(4, 1) << robot.x, robot.y, robot.z, 1.0);
        cv::Mat matrixCameraPoint{w2c * (w2r.inv() * matrixRobotPoint)};
        return getPixelFromCamera(intrinsicMatrix, matrixCameraPoint);
    }

    cv::Point2f getPixelFromCamera(const cv::Mat &intrinsicMatrix, const cv::Mat &cameraPoint)
    {
        double fx = intrinsicMatrix.at<double>(0, 0);
        double fy = intrinsicMatrix.at<double>(1, 1);
        double cx = intrinsicMatrix.at<double>(0, 2);
        double cy = intrinsicMatrix.at<double>(1, 2);
        double X = cameraPoint.at<double>(0, 0);
        double Y = cameraPoint.at<double>(1, 0);
        double Z = cameraPoint.at<double>(2, 0);
        double u = (fx * X + cx * Z) / Z;
        double v = (fy * Y + cy * Z) / Z;
        return cv::Point2f(u, v);
    }

    std::pair<double, double> RuneSolver::getPitchYawFromRobotCoor(const cv::Point3f &target, double bulletSpeed)
    {
        double yaw{utils::radian2Angle(-std::atan2(target.x, target.z)) + rune_solver_params.COMPANSATE_YAW};
        double horizontal{utils::pointPointDistance({0.0, 0.0}, {target.x, target.z})};
        double a{-0.5 * rune_solver_params.gravity * std::pow(horizontal, 2) / std::pow(bulletSpeed, 2)};
        double b{horizontal};
        double c{a + target.y};
        double result{utils::solveQuadraticEquation(a, b, c).second};
        double pitch{utils::radian2Angle(std::atan(result)) + rune_solver_params.COMPANSATE_PITCH};

        return std::make_pair(pitch, yaw);
    }

    bool RuneSolver::calculate(std::vector<cv::Point2f> &cameraPoints, rclcpp::Time timestamp)
    {
        preprocess(cameraPoints, timestamp);
        if (matrixCal() == false)
        {
            return false;
        }
        setFirstDetect();
        angleCal();
        directionCal();
        if (m_direction == Direction::UNKNOWN)
        {
            return false;
        }
        // if (predict() == false)
        // {
        //     return false;
        // }
        return true;
    }

    void RuneSolver::angleCal()
    {
        // 计算相对于第一次检测旋转的角度 angelAbs
        cv::Mat rMatRel{m_rMatW2RBase.inv() * m_rMatW2R};
        double angleAbs{-std::atan2(rMatRel.at<double>(0, 1), rMatRel.at<double>(0, 0))};
        // 减去上一次得到的角度，得到相对于上一次检测旋转的角度
        double angleMinus{angleAbs - m_angleLast};
        m_angleLast = angleAbs;
        // 用这个角度除以两片扇叶的夹角，得到装甲板切换数，并计算总的装甲板切换数
        int shift = std::round(angleMinus / ANGLE_BETWEEN_FAN_BLADES);
        m_totalShift += shift;
        // 用 angelAbs 减去装甲板切换的角度，即可得到连续的角度
        m_angleRel = angleAbs - m_totalShift * ANGLE_BETWEEN_FAN_BLADES;
        // KF TODO
        double time = (m_timestamp - m_startTime).seconds();
        // 存储相对于第一次识别的时间间隔和角度的绝对值，日后进行拟合
        if (rune_solver_params.rune_mode == RuneMode::BIG)
        {
            std::unique_lock lock(m_mutex);
            m_fitData.emplace_back(time, std::abs(m_angleRel));
        }
    }

    rm_interfaces::msg::GimbalCmd RuneSolver::solveGimbalCmd(const Eigen::Vector3d &target)
    {
        double current_yaw = 0.0, current_pitch = 0.0;

        try
        {
            auto gimbal_tf = tf2_buffer_->lookupTransform("odom", "gimbal_link", tf2::TimePointZero);
            auto msg_q = gimbal_tf.transform.rotation;

            tf2::Quaternion tf_q;
            tf2::fromMsg(msg_q, tf_q);
            double roll;
            tf2::Matrix3x3(tf_q).getRPY(roll, current_pitch, current_yaw);
            current_pitch = -current_pitch;
        }
        catch (tf2::TransformException &ex)
        {
            IMCA_ERROR("rune_solver", "{}", ex.what());
            throw ex;
        }
        // Calculate yaw and pitch
        double yaw = atan2(target.y(), target.x());
        double pitch = atan2(target.z(), target.head(2).norm());

        // Set parameters of compensator
        trajectory_compensator->velocity = rune_solver_params.bullet_speed;
        trajectory_compensator->gravity = rune_solver_params.gravity;
        trajectory_compensator->iteration_times = 30;

        if (double temp_pitch = pitch; trajectory_compensator->compensate(target, temp_pitch))
        {
            pitch = temp_pitch;
        }
        double distance = target.norm();
        double cmd_yaw = angles::normalize_angle(yaw + rune_solver_params.COMPANSATE_YAW);
        double cmd_pitch = pitch + rune_solver_params.COMPANSATE_PITCH;
        rm_interfaces::msg::GimbalCmd gimbal_cmd;
        gimbal_cmd.distance = distance;
        gimbal_cmd.yaw_diff = utils::radian2Angle(cmd_yaw - current_yaw);
        gimbal_cmd.pitch_diff = utils::radian2Angle(cmd_pitch - current_pitch);
        gimbal_cmd.yaw = cmd_yaw;
        gimbal_cmd.pitch = cmd_pitch;

        // Judge whether to shoot
        constexpr double TARGET_RADIUS = 0.308;
        double shooting_range_yaw = std::abs(atan2(TARGET_RADIUS / 2, m_distance2Target)) * 180 / M_PI;
        double shooting_range_pitch = std::abs(atan2(TARGET_RADIUS / 2, m_distance2Target)) * 180 / M_PI;
        // Limit the shooting area to 1 degree to avoid not shooting when distance is
        // too large
        shooting_range_yaw = std::max(shooting_range_yaw, 1.0);
        shooting_range_pitch = std::max(shooting_range_pitch, 1.0);
        if (std::abs(gimbal_cmd.yaw_diff) < shooting_range_yaw &&
            std::abs(gimbal_cmd.pitch_diff) < shooting_range_pitch)
        {
            gimbal_cmd.fire_advice = true;
            IMCA_DEBUG("rune_solver", "You Can Fire!");
        }
        else
        {
            gimbal_cmd.fire_advice = false;
        }
        return gimbal_cmd;
    }

    /**
     * @brief 坐标系变换
     */
    bool RuneSolver::matrixCal()
    {
        m_matW2C = world2Camera(); // pnp
        m_matC2R = camera2Robot();
        if (m_matC2R.empty())
        {
            return false;
        }
        m_matW2R = m_matC2R * m_matW2C;
        m_rMatW2R = m_matW2R(cv::Rect(0, 0, 3, 3));
        m_distance2Target = cv::norm(m_matW2C.col(3));
        if (utils::inRange<double>(m_distance2Target, rune_solver_params.MIN_DISTANCE_TO_TARGET, rune_solver_params.MAX_DISTANCE_TO_TARGET) ==
            false)
        {
            return false;
        }
        cv::Mat armorWorld = (cv::Mat_<double>(4, 1) << 0, 0, 0, 1);
        cv::Mat centerWorld = (cv::Mat_<double>(4, 1) << 0, POWER_RUNE_RADIUS, 0, 1);
        cv::Mat armorRobot{m_matW2R * armorWorld};
        cv::Mat centerCamera{m_matW2C * centerWorld};
        cv::Mat centerRobot{m_matW2R * centerWorld};
        m_armorRobot = {(float)(armorRobot.at<double>(0, 0)), (float)(armorRobot.at<double>(1, 0)),
                        (float)(armorRobot.at<double>(2, 0))};
        m_centerRobot = {(float)(centerRobot.at<double>(0, 0)), (float)(centerRobot.at<double>(1, 0)),
                         (float)(centerRobot.at<double>(2, 0))};
        return true;
    }

    /**
     * @brief 相机坐标系转机器人坐标系
     */
    cv::Mat RuneSolver::camera2Robot()
    {
        cv::Mat transform_matrix = cv::Mat::eye(4, 4, CV_64F);
        try
        {
            geometry_msgs::msg::TransformStamped gimbal_tf = tf2_buffer_->lookupTransform(
                "odom",                   // 目标坐标系
                "camera_optical_frame",   // 源坐标系
                m_timestamp,              // 请求的时间戳（自动插值）
                tf2::durationFromSec(0.02) // 超时时间
            );

            auto tf_t = gimbal_tf.transform.translation;
            auto msg_q = gimbal_tf.transform.rotation;
            tf2::Quaternion tf_q;
            tf2::fromMsg(msg_q, tf_q);
            tf2::Matrix3x3 tf_r(tf_q);
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    transform_matrix.at<double>(i, j) = tf_r[i][j];
                }
            }

            transform_matrix.at<double>(0, 3) = tf_t.x;
            transform_matrix.at<double>(1, 3) = tf_t.y;
            transform_matrix.at<double>(2, 3) = tf_t.z;
            return transform_matrix;
        }
        catch (tf2::TransformException &ex)
        {
            IMCA_ERROR("rune_solver", "{}", ex.what());
            throw ex;
            return cv::Mat();
        }
    }

    /**
     * @brief 拟合线程的主函数
     */
    void RuneSolver::fit()
    {
        if (rune_solver_params.rune_mode != RuneMode::BIG)
        {
            IMCA_WARN("rune_solver", "fit thread return");
            return;
        }
        decltype(m_fitData) fitData;
        while (STOP_THREAD.load() == false && rclcpp::ok())
        {
            {
                std::shared_lock lock(m_mutex);
                fitData = m_fitData;
            }
            // 数据量过少时，直接返回
            if (m_fitData.size() < (size_t)rune_solver_params.MIN_FIT_DATA_SIZE)
            {
                continue;
            }
            bool result = fitOnce();
            VALID_PARAMS.store(result);

            MUTEX.lock();
            if (result == true)
            {
                // IMCA_INFO("rune_solver","--------------------------------");
                // std::cout << "params: ";
                // std::for_each(m_params.begin(), m_params.end(), [](auto &&it)
                //               { std::cout << it << " "; });
                // std::cout << std::endl;
            }
            MUTEX.unlock();

            if (m_fitData.size() > (size_t)rune_solver_params.MAX_FIT_DATA_SIZE)
            {
                m_fitData.erase(m_fitData.begin(), m_fitData.begin() + m_fitData.size() / 2);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(int(1e4 / FPS)));
        }
    }

    /**
     * @brief 拟合一次
     */
    bool RuneSolver::fitOnce()
    {
        // 如果数据量过少，则确定凹凸性
        if (m_fitData.size() < (size_t)2 * rune_solver_params.MIN_FIT_DATA_SIZE)
        {
            m_convexity = getConvexity(m_fitData);
        }
        // 利用 ransac 算法计算参数
        m_params = ransacFitting(m_fitData, m_convexity);
        return true;
    }

    /**
     * @brief 凹凸性计算
     * @param[in] data          角度数据
     * @return Convexity
     */
    Convexity getConvexity(const std::vector<std::pair<double, double>> &data)
    {
        auto first{data.begin()}, last{data.end() - 1};
        double slope{(last->second - first->second) / (last->first - first->first)};
        double offset{(first->second * last->first - last->second * first->first) / (last->first - first->first)};
        int concave{0}, convex{0};
        for (const auto &i : data)
        {
            if (slope * i.first + offset > i.second)
            {
                concave++;
            }
            else
            {
                convex++;
            }
        }
        const int standard{static_cast<int>(data.size() * 0.75)};
        return concave > standard  ? Convexity::CONCAVE
               : convex > standard ? Convexity::CONVEX
                                   : Convexity::UNKNOWN;
    }

    /**
     * @brief ransac 算法，返回拟合参数
     * @param[in] data          角度数据
     * @param[in] convexity     凹凸性
     * @return std::array<double, 5>
     */
    std::array<double, 5> ransacFitting(const std::vector<std::pair<double, double>> &data, Convexity convexity)
    {
        // inliers 为符合要求的点，outliers 为不符合要求的点
        std::vector<std::pair<double, double>> inliers, outliers;
        // 初始时，inliers 为全部点
        inliers.assign(data.begin(), data.end());
        // 迭代次数
        int iterTimes{data.size() < 400 ? 200 : 20};
        // 初始参数
        std::array<double, 5> params{0.470, 1.942, 0, 1.178, 0};
        for (int i = 0; i < iterTimes; ++i)
        {
            decltype(inliers) sample;
            // 如果数据点较多，则将数据打乱，取其中一部分
            if (inliers.size() > 400)
            {
                std::shuffle(
                    inliers.begin(), inliers.end() - 100,
                    std::default_random_engine(std::chrono::system_clock::now().time_since_epoch().count()));
                sample.assign(inliers.end() - 200, inliers.end());
            }
            else
            {
                sample.assign(inliers.begin(), inliers.end());
            }
            // 进行拟合
            params = leastSquareEstimate(sample, params, convexity);
            // 对 inliers 每一个计算误差
            std::vector<double> errors;
            for (const auto &inlier : inliers)
            {
                errors.push_back(std::abs(inlier.second - getAngleBig(inlier.first, params)));
            }
            // 如果数据量较大，则对点进行筛选
            if (data.size() > 800)
            {
                std::sort(errors.begin(), errors.end());
                const int index{static_cast<int>(errors.size() * 0.95)};
                const double threshold{errors[index]};
                // 剔除 inliers 中不符合要求的点
                for (size_t i = 0; i < inliers.size() - 100; ++i)
                {
                    if (std::abs(inliers[i].second - getAngleBig(inliers[i].first, params)) > threshold)
                    {
                        outliers.push_back(inliers[i]);
                        inliers.erase(inliers.begin() + i);
                    }
                }
                // 将 outliers 中符合要求的点加进来
                for (size_t i = 0; i < outliers.size(); ++i)
                {
                    if (std::abs(outliers[i].second - getAngleBig(outliers[i].first, params)) < threshold)
                    {
                        inliers.emplace(inliers.begin(), outliers[i]);
                        outliers.erase(outliers.begin() + i);
                    }
                }
            }
        }
        // 返回之前对所有 inliers 再拟合一次
        return params;
    }

    /**
     * @brief 最小二乘拟合，返回参数列表
     * @param[in] points        数据点
     * @param[in] params        初始参数
     * @param[in] convexity     凹凸性
     * @return std::array<double, 5>
     */
    std::array<double, 5> leastSquareEstimate(const std::vector<std::pair<double, double>> &points,
                                              const std::array<double, 5> &params, Convexity convexity)
    {
        std::array<double, 5> ret = params;
        ceres::Problem problem;
        for (size_t i = 0; i < points.size(); i++)
        {
            ceres::CostFunction *costFunction = new CostFunctor2(points[i].first, points[i].second);
            ceres::LossFunction *lossFunction = new ceres::SoftLOneLoss(0.1);
            problem.AddResidualBlock(costFunction, lossFunction, ret.begin());
        }
        std::array<double, 3> omega;
        if (points.size() < 100)
        {
            // 在数据量较小时，可以利用凹凸性定参数边界
            if (convexity == Convexity::CONCAVE)
            {
                problem.SetParameterUpperBound(ret.begin(), 2, -2.8);
                problem.SetParameterLowerBound(ret.begin(), 2, -4);
            }
            else if (convexity == Convexity::CONVEX)
            {
                problem.SetParameterUpperBound(ret.begin(), 2, -1.1);
                problem.SetParameterLowerBound(ret.begin(), 2, -2.3);
            }
            omega = {10., 1., 1.};
        }
        else
        {
            // 而数据量较多后，则不再需要凹凸性辅助拟合
            omega = {60., 50., 50.};
        }
        ceres::CostFunction *costFunction1 = new CostFunctor1(ret[0], 0);
        ceres::LossFunction *lossFunction1 =
            new ceres::ScaledLoss(new ceres::HuberLoss(0.1), omega[0], ceres::TAKE_OWNERSHIP);
        problem.AddResidualBlock(costFunction1, lossFunction1, ret.begin());
        ceres::CostFunction *costFunction2 = new CostFunctor1(ret[1], 1);
        ceres::LossFunction *lossFunction2 =
            new ceres::ScaledLoss(new ceres::HuberLoss(0.1), omega[1], ceres::TAKE_OWNERSHIP);
        problem.AddResidualBlock(costFunction2, lossFunction2, ret.begin());
        ceres::CostFunction *costFunction3 = new CostFunctor1(ret[3], 3);
        ceres::LossFunction *lossFunction3 =
            new ceres::ScaledLoss(new ceres::HuberLoss(0.1), omega[2], ceres::TAKE_OWNERSHIP);
        problem.AddResidualBlock(costFunction3, lossFunction3, ret.begin());
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 50;
        options.minimizer_progress_to_stdout = false;
        options.check_gradients = false;
        options.gradient_check_relative_precision = 1e-4;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        return ret;
    }

    cv::Mat RuneSolver::world2Camera()
    {
        cv::Mat rVec, tVec, rMat;
        pnp_solver->solvePnP(m_cameraPoints, rVec, tVec, "rune");

        cv::Rodrigues(rVec, rMat);
        cv::Mat w2c{cv::Mat::zeros(cv::Size(4, 4), CV_64FC1)};
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                w2c.at<double>(i, j) = rMat.at<double>(i, j);
            }
        }
        for (int i = 0; i < 3; ++i)
        {
            w2c.at<double>(i, 3) = tVec.at<double>(i, 0);
        }
        w2c.at<double>(3, 3) = 1.0;
        return w2c;
    }
} // namespace imca::rune
