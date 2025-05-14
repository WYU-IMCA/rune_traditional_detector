#ifndef RUNE_SOLVER_RUNE_SOLVER_HPP_
#define RUNE_SOLVER_RUNE_SOLVER_HPP_

#include <atomic>
#include <thread>
#include <random>
#include <ceres/ceres.h>
#include <opencv2/opencv.hpp>

// ros2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// project
#include "rune_solver/types.hpp"
#include "rm_utils/math/pnp_solver.hpp"
#include "rm_utils/math/trajectory_compensator.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/math/utils.hpp"
#include "rm_interfaces/msg/gimbal_cmd.hpp"
namespace imca::rune
{
    
    Convexity getConvexity(const std::vector<std::pair<double, double>> &data);
    std::array<double, 5> ransacFitting(const std::vector<std::pair<double, double>> &data, Convexity convexity);
    std::array<double, 5> leastSquareEstimate(const std::vector<std::pair<double, double>> &points,
                                              const std::array<double, 5> &params, Convexity convexity);

    cv::Point2f getPixelFromRobot(const cv::Mat &intrinsicMatrix, const cv::Point3f &robot, const cv::Mat &w2c, const cv::Mat &w2r);
    cv::Point2f getPixelFromCamera(const cv::Mat &intrinsicMatrix, const cv::Mat &cameraPoint);

    class RuneSolver
    {
    public:
        std::atomic<bool> VALID_PARAMS;
        struct RuneSolverParams
        {
            std::string compensator_type;
            RuneMode rune_mode;
            double gravity;
            double bullet_speed;
            double MIN_DISTANCE_TO_TARGET;
            double MAX_DISTANCE_TO_TARGET;
            int MIN_FIT_DATA_SIZE;
            int MAX_FIT_DATA_SIZE;
            double COMPANSATE_TIME;  // ms
            double COMPANSATE_PITCH; // rad
            double COMPANSATE_YAW;
        };
        RuneSolver(RuneSolverParams params, std::shared_ptr<tf2_ros::Buffer> tf2_buffer);
        ~RuneSolver();
        std::unique_ptr<PnPSolver> pnp_solver;

        RuneSolverParams rune_solver_params;
        bool calculate(std::vector<cv::Point2f> &cameraPoints, rclcpp::Time timestamp);
        rm_interfaces::msg::GimbalCmd solveGimbalCmd(const Eigen::Vector3d &target);
        std::unique_ptr<TrajectoryCompensator> trajectory_compensator;
        cv::Point3f getPredictPoint() const noexcept
        {
            return m_predictRobot;
        }
        Eigen::Vector3d getCurPose() const noexcept
        {
            return Eigen::Vector3d(
                m_armorRobot.x,
                m_armorRobot.y,
                m_armorRobot.z);
        }
        Eigen::Vector3d getCurCenterPose() const noexcept
        {
            return Eigen::Vector3d(
                m_centerRobot.x,
                m_centerRobot.y,
                m_centerRobot.z);
        }
        cv::Point2f getPredictPixel() const noexcept{
            return m_predictPixel;
        }
        double preidctTarget(Eigen::Vector3d &pred_pos, double timestamp);
        void startFitThread()
        {
            std::lock_guard<std::mutex> lock(m_threadMutex);
            if (!m_fitThread.joinable())
            {
                STOP_THREAD.store(false);
                m_fitThread = std::thread(&RuneSolver::fit, this);
                IMCA_INFO("rune_solver", "线程已启动");
            }
        }
        void stopFitThread()
        {
            std::lock_guard<std::mutex> lock(m_threadMutex);
            if (m_fitThread.joinable())
            {
                STOP_THREAD.store(true);
                m_fitThread.join();
                m_fitData.clear();
                VALID_PARAMS.store(false);
                IMCA_INFO("rune_solver", "线程已停止");
            }
        }

    private:
        std::mutex MUTEX;
        std::atomic<bool> STOP_THREAD;
        std::mutex m_threadMutex;
        std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
        std::vector<cv::Point2f> m_cameraPoints;
        rclcpp::Time m_timestamp; // 当前帧的时间戳
        rclcpp::Time m_startTime; // 开始的时间戳
        Direction m_direction;    // 旋转方向
        Convexity m_convexity;    // 拟合数据凹凸性
        int m_totalShift;         // 总体的装甲板切换数
        bool m_firstDetect;       // 第一次检测的标志位，第一次检测有效之后置为 true
        double m_angleRel;        // 这一帧相对于第一帧的旋转角度（去除了装甲板切换的影响）
        double m_angleLast;       // 上一帧相对于第一帧的旋转角度（不考虑装甲板切换
        double m_distance2Target;
        std::vector<std::pair<double, double>> m_fitData; // 拟合数据
        std::vector<double> m_directionData;              // 计算旋转方向的数据
        std::array<double, 5> m_params;                   // 拟合参数
        int m_directionThresh;

        cv::Mat m_matW2C;           // 世界坐标系转相机坐标系的 4x4 变换矩阵
        cv::Mat m_matC2R;           // 相机坐标系转机器人坐标系的 4x4 变换矩阵
        cv::Mat m_matW2R;           // 世界坐标系转机器人坐标系的 4x4 变换矩阵
        cv::Mat m_rMatW2RBase;      // 第一次检测有效的世界坐标系转机器人坐标系的 3x3 旋转矩阵
        cv::Mat m_rMatW2R;          // 世界坐标系转机器人坐标系的 3x3 旋转矩阵
        cv::Point3f m_armorRobot;   // 装甲板中心的机器人坐标
        cv::Point3f m_centerRobot;  // 中心 R 的机器人坐标
        cv::Point3f m_predictRobot; // 预测击打的机器人坐标
        cv::Point2f m_predictPixel; // 预测击打的像素坐标
        std::thread m_fitThread;
        std::shared_mutex m_mutex;

        double m_predictPitch;
        double m_predictYaw;

        void fit();
        bool fitOnce();
        cv::Mat world2Camera();
        cv::Mat camera2Robot();
        bool matrixCal();
        void preprocess(std::vector<cv::Point2f> &cameraPoints, rclcpp::Time timestamp);
        void setFirstDetect();
        void angleCal();
        void directionCal();
        bool predict();
        std::pair<double, double> getPitchYawFromRobotCoor(const cv::Point3f &target, double bulletSpeed);
    };

    /**
     * @brief 惩罚项，让拟合的参数更加贴近预设的参数
     */
    class CostFunctor1 : public ceres::SizedCostFunction<1, 5>
    {
    public:
        CostFunctor1(double truth_, int id_) : truth(truth_), id(id_) {}
        virtual ~CostFunctor1() {};
        virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
        {
            double pre = parameters[0][id];
            residuals[0] = pre - truth;
            if (jacobians != nullptr)
            {
                if (jacobians[0] != nullptr)
                {
                    for (int i = 0; i < 5; ++i)
                    {
                        if (i == id)
                        {
                            jacobians[0][i] = 1;
                        }
                        else
                        {
                            jacobians[0][i] = 0;
                        }
                    }
                }
            }
            return true;
        }
        double truth;
        int id;
    };

    /**
     * @brief 拟合项
     */
    class CostFunctor2 : public ceres::SizedCostFunction<1, 5>
    {
    public:
        CostFunctor2(double t_, double y_) : t(t_), y(y_) {}
        virtual ~CostFunctor2() {};
        virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
        {
            double a = parameters[0][0];
            double w = parameters[0][1];
            double t0 = parameters[0][2];
            double b = parameters[0][3];
            double c = parameters[0][4];
            double cs = cos(w * (t + t0));
            double sn = sin(w * (t + t0));
            residuals[0] = -a * cs + b * t + c - y;
            if (jacobians != NULL)
            {
                if (jacobians[0] != NULL)
                {
                    jacobians[0][0] = -cs;
                    jacobians[0][1] = a * (t + t0) * sn;
                    jacobians[0][2] = a * w * sn;
                    jacobians[0][3] = t;
                    jacobians[0][4] = 1;
                }
            }
            return true;
        }
        double t, y;
    };

    /**
     * @brief 得到小符旋转角
     * @param[in] distance      到装甲板中心的距离
     * @param[in] bulletSpeed   弹速
     * @param[in] PMSpeed       小符旋转速度
     * @param[in] compansate    时间补偿
     * @return double
     */
    inline double getRotationAngleSmall(double distance, double bulletSpeed, double rotationSpeed,
                                        double compansate) noexcept
    {
        return rotationSpeed * (distance / bulletSpeed + compansate / 1e3);
    }

    /**
     * @brief 得到大符角度，注意这里是利用参数计算出来的，相对于第一次识别的角度
     * @param[in] time          时间
     * @param[in] params        参数
     * @return double
     */
    inline double getAngleBig(double time, const std::array<double, 5> &params) noexcept
    {
        return -params[0] * std::cos(params[1] * (time + params[2])) + params[3] * time + params[4];
    }

    /**
     * @brief 得到大符旋转角度，这里是相对于最后一次识别，预测的旋转角
     * @param[in] distance      到装甲板中心的距离
     * @param[in] bulletSpeed   弹速
     * @param[in] params        参数
     * @param[in] compansate    补偿
     * @param[in] frameTime     最后一次识别的时间戳
     * @return double
     */
    inline double getRotationAngleBig(double distance, double bulletSpeed, const std::array<double, 5> &params,
                                      double compansate, int64_t frameTime) noexcept
    {
        double predictTime{distance / bulletSpeed + (frameTime + compansate) * 1e-3};
        double currentTime{frameTime * 1e-3};
        return getAngleBig(predictTime, params) - getAngleBig(currentTime, params);
    }

} // namespace imca::rune
#endif
