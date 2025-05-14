#ifndef RUNE_SOLVER_TYPES_HPP_
#define RUNE_SOLVER_TYPES_HPP_

// STD
#include <array>

// 3rd party
#include <ceres/ceres.h>

#include <opencv2/opencv.hpp>

namespace imca::rune
{
    enum class Direction
    {
        UNKNOWN,
        STABLE,
        ANTI_CLOCKWISE,
        CLOCKWISE
    }; // 旋转方向
    enum class Convexity
    {
        UNKNOWN,
        CONCAVE,
        CONVEX
    }; // 拟合曲线凹凸性
    enum class RuneMode
    {
        SMALL,
        BIG
    }; // 模式：小符，大符

    constexpr double POWER_RUNE_RADIUS = 700.0 * 1e-3, LEAF_RADIUS{150.0 * 1e-3};
    inline static const double ANGLE_BETWEEN_FAN_BLADES{72 * CV_PI / 180};
    inline static const double SMALL_POWER_RUNE_ROTATION_SPEED{1.04719};

    constexpr int FPS = 169;

    const std::vector<cv::Point3f> RUNE_OBJECT_POINTS =
        {{0.0, -(float)LEAF_RADIUS, 0.0},
         {(float)LEAF_RADIUS, 0.0, 0.0},
         {0.0, (float)LEAF_RADIUS, 0.0},
         {-(float)LEAF_RADIUS, 0.0, 0.0},
         {0.0, (float)POWER_RUNE_RADIUS, 0.0}};

    // const std::vector<cv::Point3f> RUNE_OBJECT_POINTS =
    //     {{0.0, 0.0, (float)LEAF_RADIUS},
    //      {0.0, -(float)LEAF_RADIUS, 0.0},
    //      {0.0, 0.0, -(float)LEAF_RADIUS},
    //      {0.0, (float)LEAF_RADIUS, 0.0},
    //      {0.0, -(float)POWER_RUNE_RADIUS, 0.0}};

} // namespace imca::rune

#endif // RUNE_SOLVER_TYPES_HPP_