#ifndef RUNE_DETECTOR_TYPES_HPP_
#define RUNE_DETECTOR_TYPES_HPP_
// 3rd party
#include <opencv2/opencv.hpp>
// project
#include "rm_utils/common.hpp"
namespace imca::rune
{

    // params
    const int MAX_BRIGHTNESS{255};
    const int IMAGE_WIDTH{1440}, IMAGE_HEIGHT{1080};

    inline static const float ARMOR_CENTER_VERTICAL_DISTANCE_THRESHOLD{90};
    inline static const double GLOBAL_ROI_LENGTH_RATIO{1.5};
    inline static const double POINT_POINT_THETA_THRESHOLD_MIN = 80.;
    inline static const double POINT_POINT_THETA_THRESHOLD_MAX = 100.;
    inline static const double LEAF_RADIUS = 150.; // mm
    inline static const double POWER_RUNE_RADIUS{700.0};

    enum class Status
    {
        SUCCESS,
        ARROW_FAILURE,
        ARMOR_FAILURE,
        CENTER_FAILURE
    }; // 成功，箭头检测失败，装甲板检测失败，中心R检测失败

    struct PolarPoint
    {
        PolarPoint() = default;
        PolarPoint(double theta, double rho, cv::Point2f pt) : theta(theta), rho(rho), pt(pt) {}
        double theta;   // 极角
        double rho;     // 半径
        cv::Point2f pt; // 原始点坐标
    };

    inline double pointLineDistance(const cv::Point2f &pt, const cv::Vec4f &line);

    /**
     * @brief 灯条
     */
    struct Lightline
    {
        Lightline() = default;
        Lightline(const std::vector<cv::Point> &contour, const cv::Rect2f &localRoi,
                  const cv::Rect2f &globalRoi = cv::Rect2f(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT)) : m_contour(contour), m_contourArea(cv::contourArea(contour)), m_rotatedRect(cv::minAreaRect(contour))
        {
            // 长的为 length，短的为 width
            m_width = m_rotatedRect.size.width, m_length = m_rotatedRect.size.height;
            if (m_width > m_length)
            {
                std::swap(m_width, m_length);
            }
            m_aspectRatio = m_length / m_width;
            m_center = m_rotatedRect.center;
            m_angle = m_rotatedRect.angle;
            m_area = m_rotatedRect.size.width * m_rotatedRect.size.height;
            std::array<cv::Point2f, 4> points;
            m_rotatedRect.points(points.begin());
            /**
             * OpenCV 中 RotatedRect::points() 角点顺序为顺时针，p[0]
             * 为纵坐标最大的点。若有多个纵坐标最大，则取其中横坐标最大的点。 p[0] 到 p[3] 的边为 width，其邻边为
             * height。 根据上述关系可以确立四个角点位置。如果是装甲板灯条，则其还需要结合中心 R 来得到中心 R
             * 参照下的角点位置。
             */
            if (m_rotatedRect.size.width > m_rotatedRect.size.height)
            {
                m_tl = points[1];
                m_tr = points[2];
                m_bl = points[0];
                m_br = points[3];
            }
            else
            {
                m_tl = points[0];
                m_tr = points[1];
                m_bl = points[3];
                m_br = points[2];
            }
            // 得到相对原图的角点和中心位置
            m_tl += localRoi.tl() + globalRoi.tl();
            m_tr += localRoi.tl() + globalRoi.tl();
            m_bl += localRoi.tl() + globalRoi.tl();
            m_br += localRoi.tl() + globalRoi.tl();
            m_center += localRoi.tl() + globalRoi.tl();
            m_x = m_center.x, m_y = m_center.y;
        };
        std::vector<cv::Point> m_contour; // 轮廓点集
        double m_contourArea;             // 轮廓面积
        double m_area;                    // 外接旋转矩形面积
        cv::RotatedRect m_rotatedRect;    // 外接旋转矩形
        cv::Point2f m_tl;                 // 左上角点
        cv::Point2f m_tr;                 // 右上角点
        cv::Point2f m_bl;                 // 左下角点
        cv::Point2f m_br;                 // 右下角点
        cv::Point2f m_center;             // 中心点
        double m_length;                  // 长度
        double m_width;                   // 宽度
        double m_x;                       // 中心点 x 坐标
        double m_y;                       // 中心点 y 坐标
        double m_angle;                   // 旋转矩形角度
        double m_aspectRatio;             // 旋转矩形长宽比
    };

    /**
     * @brief 装甲板
     */
    struct Armor
    {
        Armor() = default;
        void set(std::vector<PolarPoint> &pt, cv::Point2f &center)
        {
            points = pt;
            m_center = center;
            m_top = points[0].pt;
            m_right = points[1].pt;
            m_in = points[2].pt;
            m_left = points[3].pt;
            m_radius = utils::pointPointDistance(m_top, m_center);
        }
        cv::Point2f m_top;
        cv::Point2f m_in;
        cv::Point2f m_left;
        cv::Point2f m_right;
        cv::Point2f m_center; // 装甲板中心点
        std::vector<PolarPoint> points;
        double m_x; // 装甲板中心 x 坐标
        double m_y; // 装甲板中心 y 坐标
        double m_radius;
    };

    struct ArmorContour
    {
        ArmorContour() = default;
        ArmorContour(const std::vector<cv::Point> &contour, const cv::Rect2f &localRoi,
                     const cv::Rect2f &globalRoi = cv::Rect2f(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT))
            : m_contour(contour),
              localRoi(localRoi),
              globalRoi(globalRoi),
              m_contourArea(cv::contourArea(contour)),
              m_rotatedRect(cv::minAreaRect(contour)),
              M(cv::moments(contour))
        {
            // 长的为 length，短的为 width
            m_width = m_rotatedRect.size.width, m_length = m_rotatedRect.size.height;
            if (m_width > m_length)
            {
                std::swap(m_width, m_length);
            }
            m_aspectRatio = m_length / m_width;
            m_angle = m_rotatedRect.angle;
            center = cv::Point2f(M.m10 / M.m00, M.m01 / M.m00);
        }
        std::vector<cv::Point> m_contour; // 轮廓点集
        cv::Rect2f localRoi;
        cv::Rect2f globalRoi;
        double m_contourArea;          // 轮廓面积
        cv::RotatedRect m_rotatedRect; // 外接旋转矩形
        double m_length;               // 长度
        double m_width;                // 宽度
        double m_angle;                // 旋转矩形角度
        double m_aspectRatio;          // 旋转矩形长宽比
        cv::Moments M;
        cv::Point2f center;                  // Moment center in point2f
        std::vector<PolarPoint> polarPoints; // get 4 points finnally
        bool calcSrcPointsAndCenter()
        {
            for (auto &&point : polarPoints)
            {
                point.pt += localRoi.tl() + globalRoi.tl();
            }
            center += localRoi.tl() + globalRoi.tl();

            return true;
        };
    };

    struct NewArmor
    {
        NewArmor() = default;
        void set(std::vector<PolarPoint> &pt, cv::Point2f &center)
        {
            points = pt;
        m_center = center;
        m_top = points[0].pt;
        m_right = points[1].pt;
        m_in = points[2].pt;
        m_left = points[3].pt;
        double tmp{0};
        for (auto&& i : pt) {
            tmp += utils::pointPointDistance(i.pt, m_center);
        }
        m_radius = tmp / 4.0;
        }
        cv::Point2f m_top;
        cv::Point2f m_in;
        cv::Point2f m_left;
        cv::Point2f m_right;
        cv::Point2f m_center; // 装甲板中心点
        std::vector<PolarPoint> points;
        double m_x; // 装甲板中心 x 坐标
        double m_y; // 装甲板中心 y 坐标
        double m_radius;
    };

    /**
     * @brief 中心 R
     */
    struct CenterR
    {
        CenterR() = default;
        void set(const Lightline &lightline)
        {
            m_lightline = lightline;
            m_boundingRect = cv::boundingRect(lightline.m_contour);
            // 由于灯条角点和中心点已经设置过 roi，因此这里不需要重新设置
            m_center = lightline.m_center;
            m_x = m_center.x, m_y = m_center.y;
            return;
        };
        Lightline m_lightline;   // 中心 R 灯条
        cv::Point2f m_center;    // 中心 R 点
        cv::Rect m_boundingRect; // 中心 R 最小正矩形
        double m_x;              // 中心 R x 坐标
        double m_y;              // 中心 R y 坐标
    };

    /**
     * @brief 箭头
     */
    struct Arrow
    {
        Arrow() = default;
        void set(const std::vector<Lightline> &lightlines, const cv::Point2f &roi)
        {
            std::vector<cv::Point2f> arrowPoints;
            double fillArea = 0.0;
            double pointLineThresh = 0.0;
            std::for_each(lightlines.begin(), lightlines.end(), [&](const Lightline &l)
                          {
        arrowPoints.insert(arrowPoints.end(), l.m_contour.begin(), l.m_contour.end());
        fillArea += l.m_contourArea;
        pointLineThresh += l.m_length / lightlines.size(); });
            // 滤除距离较大的点
            m_contour.clear();
            cv::Vec4f line;
            cv::fitLine(arrowPoints, line, cv::DIST_L2, 0, 0.01, 0.01);
            for (const auto &point : arrowPoints)
            {
                if (pointLineDistance(point, line) < pointLineThresh)
                {
                    m_contour.push_back(point);
                }
            }
            // 设置成员变量
            m_rotatedRect = cv::minAreaRect(m_contour);
            m_center = m_rotatedRect.center + roi;
            m_length = m_rotatedRect.size.height;
            m_width = m_rotatedRect.size.width;
            // RotatedRect::angle 范围为 -90~0. 这里根据长宽长度关系，将角度扩展到 -90~90
            if (m_length < m_width)
            {
                m_angle = m_rotatedRect.angle;
                // 长的为 length
                std::swap(m_length, m_width);
            }
            else
            {
                m_angle = m_rotatedRect.angle + 90;
            }
            m_aspectRatio = m_length / m_width;
            m_area = m_length * m_width;
            m_fillRatio = fillArea / m_area;
            return;
        };
        std::vector<cv::Point> m_contour; // 轮廓点集
        cv::RotatedRect m_rotatedRect;    // 外接旋转矩形
        double m_length;                  // 长度
        double m_width;                   // 宽度
        cv::Point2f m_center;             // 中心点
        double m_angle;                   // 角度
        double m_aspectRatio;             // 长宽比
        double m_area;                    // 面积
        double m_fillRatio;               // 填充比例
    };

    /**
     * @brief Construct a new Lightline:: Lightline object
     * @param[in] contour       轮廓点集
     * @param[in] roi           roi 用来设置正确的中心及角点
     */
    // Lightline::Lightline(const std::vector<cv::Point> &contour, const cv::Rect2f &localRoi,
    //                      const cv::Rect2f &globalRoi)
    //     : m_contour(contour), m_contourArea(cv::contourArea(contour)), m_rotatedRect(cv::minAreaRect(contour))
    // {
    //     // 长的为 length，短的为 width
    //     m_width = m_rotatedRect.size.width, m_length = m_rotatedRect.size.height;
    //     if (m_width > m_length)
    //     {
    //         std::swap(m_width, m_length);
    //     }
    //     m_aspectRatio = m_length / m_width;
    //     m_center = m_rotatedRect.center;
    //     m_angle = m_rotatedRect.angle;
    //     m_area = m_rotatedRect.size.width * m_rotatedRect.size.height;
    //     std::array<cv::Point2f, 4> points;
    //     m_rotatedRect.points(points.begin());
    //     /**
    //      * OpenCV 中 RotatedRect::points() 角点顺序为顺时针，p[0]
    //      * 为纵坐标最大的点。若有多个纵坐标最大，则取其中横坐标最大的点。 p[0] 到 p[3] 的边为 width，其邻边为
    //      * height。 根据上述关系可以确立四个角点位置。如果是装甲板灯条，则其还需要结合中心 R 来得到中心 R
    //      * 参照下的角点位置。
    //      */
    //     if (m_rotatedRect.size.width > m_rotatedRect.size.height)
    //     {
    //         m_tl = points[1];
    //         m_tr = points[2];
    //         m_bl = points[0];
    //         m_br = points[3];
    //     }
    //     else
    //     {
    //         m_tl = points[0];
    //         m_tr = points[1];
    //         m_bl = points[3];
    //         m_br = points[2];
    //     }
    //     // 得到相对原图的角点和中心位置
    //     m_tl += localRoi.tl() + globalRoi.tl();
    //     m_tr += localRoi.tl() + globalRoi.tl();
    //     m_bl += localRoi.tl() + globalRoi.tl();
    //     m_br += localRoi.tl() + globalRoi.tl();
    //     m_center += localRoi.tl() + globalRoi.tl();
    //     m_x = m_center.x, m_y = m_center.y;
    // }

    /**
     * @brief 设置箭头
     * @param[in] points        点集
     * @param[in] roi
     */
    // void Arrow::set(const std::vector<Lightline> &lightlines, const cv::Point2f &roi)
    // {
    //     std::vector<cv::Point2f> arrowPoints;
    //     double fillArea = 0.0;
    //     double pointLineThresh = 0.0;
    //     std::for_each(lightlines.begin(), lightlines.end(), [&](const Lightline &l)
    //                   {
    //     arrowPoints.insert(arrowPoints.end(), l.m_contour.begin(), l.m_contour.end());
    //     fillArea += l.m_contourArea;
    //     pointLineThresh += l.m_length / lightlines.size(); });
    //     // 滤除距离较大的点
    //     m_contour.clear();
    //     cv::Vec4f line;
    //     cv::fitLine(arrowPoints, line, cv::DIST_L2, 0, 0.01, 0.01);
    //     for (const auto &point : arrowPoints)
    //     {
    //         if (pointLineDistance(point, line) < pointLineThresh)
    //         {
    //             m_contour.push_back(point);
    //         }
    //     }
    //     // 设置成员变量
    //     m_rotatedRect = cv::minAreaRect(m_contour);
    //     m_center = m_rotatedRect.center + roi;
    //     m_length = m_rotatedRect.size.height;
    //     m_width = m_rotatedRect.size.width;
    //     // RotatedRect::angle 范围为 -90~0. 这里根据长宽长度关系，将角度扩展到 -90~90
    //     if (m_length < m_width)
    //     {
    //         m_angle = m_rotatedRect.angle;
    //         // 长的为 length
    //         std::swap(m_length, m_width);
    //     }
    //     else
    //     {
    //         m_angle = m_rotatedRect.angle + 90;
    //     }
    //     m_aspectRatio = m_length / m_width;
    //     m_area = m_length * m_width;
    //     m_fillRatio = fillArea / m_area;
    //     return;
    // }
    /**
     * @brief 设置装甲板参数
     * @param[in] l1
     * @param[in] l2
     */
    // void Armor::set(const Lightline &l1, const Lightline &l2)
    // {
    //     if (l1.m_contourArea > l2.m_contourArea)
    //     {
    //         m_inside = l1;
    //         m_outside = l2;
    //     }
    //     else
    //     {
    //         m_outside = l1;
    //         m_inside = l2;
    //     }
    //     m_center = (m_inside.m_center + m_outside.m_center) * 0.5;
    //     m_x = m_center.x, m_y = m_center.y;
    //     m_tlIn = m_inside.m_tl;
    //     m_trIn = m_inside.m_tr;
    //     m_blIn = m_inside.m_bl;
    //     m_brIn = m_inside.m_br;
    //     m_tlOut = m_outside.m_tl;
    //     m_trOut = m_outside.m_tr;
    //     m_blOut = m_outside.m_bl;
    //     m_brOut = m_outside.m_br;
    //     return;
    // }

    /**
     * @brief 设置中心 R
     * @param[in] lightline
     */
    // void CenterR::set(const Lightline &lightline)
    // {
    //     m_lightline = lightline;
    //     m_boundingRect = cv::boundingRect(lightline.m_contour);
    //     // 由于灯条角点和中心点已经设置过 roi，因此这里不需要重新设置
    //     m_center = lightline.m_center;
    //     m_x = m_center.x, m_y = m_center.y;
    //     return;
    // }

    /**
     * @brief
     * 设置装甲板角点，注意大小为4，顺序必须为内部灯条左上角点、内部灯条右上角点、外部灯条左下角点、外部灯条右下角点
     * @param[in] points        输入的角点向量
     */
    // void Armor::setCornerPoints(const std::vector<cv::Point2f> &points)
    // {
    //     m_tlIn = points.at(0);
    //     m_trIn = points.at(1);
    //     m_blOut = points.at(2);
    //     m_brOut = points.at(3);
    // }

    const inline static cv::Scalar RED{0, 0, 255};
    const inline static cv::Scalar BLUE{255, 0, 0};
    const inline static cv::Scalar GREEN{0, 255, 0};
    const inline static cv::Scalar WHITE{255, 255, 255};
    const inline static cv::Scalar YELLOW{0, 255, 255};
    const inline static cv::Scalar PURPLE{128, 0, 128};

    inline static cv::Scalar DRAW_COLOR;

    /**
     * @brief 两个二维点间距离
     * @param[in] pt1
     * @param[in] pt2
     * @return double
     */
    inline double pointPointDistance(const cv::Point2f &pt1, const cv::Point2f &pt2) noexcept
    {
        return std::sqrt((pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y));
    }

    /**
     * @brief 点到直线间距离
     * @param[in] ptP
     * @param[in] ptL1
     * @param[in] ptL2
     * @return double
     */
    inline double pointLineDistance(const cv::Point2f &ptP, const cv::Point2f &ptL1,
                                    const cv::Point2f &ptL2) noexcept
    {
        double A = ptL2.y - ptL1.y;
        double B = ptL1.x - ptL2.x;
        double C = ptL2.x * ptL1.y - ptL2.y * ptL1.x;
        double distance = fabs(A * ptP.x + B * ptP.y + C) / sqrt(A * A + B * B);
        return distance;
    }

    /**
     * @brief 点到直线距离
     * @param[in] pt
     * @param[in] line
     * @return double
     */
    inline double pointLineDistance(const cv::Point2f &pt, const cv::Vec4f &line)
    {
        cv::Vec2f line_dir(line[0], line[1]);
        cv::Point2f line_pt(line[2], line[3]);
        return cv::norm((pt - line_pt).cross(line_dir));
    }
} // namespace imca::rune

#endif