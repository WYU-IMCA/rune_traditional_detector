// std
#include <algorithm>
#include <array>
#include <filesystem>
#include <numeric>
#include <vector>
// project
#include "rm_utils/common.hpp"
#include "rm_utils/math/utils.hpp"
#include "rune_detector/types.hpp"
#include "rm_utils/logger/log.hpp"

namespace imca::rune
{
    bool inRect(const cv::Point2f &point, const cv::Rect2f &rect);

    class RuneDetector
    {
    public:
        struct ArrowParams
        {
            int blue_brightness_threshold;
            int red_brightness_threshold;
            struct lightline
            {
                double area_min;
                double area_max;
                double aspect_ratio_max;
                int num_min;
                int num_max;
            } lightline;
            double same_area_ratio_max;
            double aspect_ratio_min;
            double aspect_ratio_max;
            double area_max;
        };
        struct ArmorParams
        {
            int blue_brightness_threshold;
            int red_brightness_threshold;
            double armor_contour_area_min;
            double armor_contour_area_max;
            double area_ratio_min;
            double area_ratio_max;
            int armor_center_vertical_distance_threshold;
        };

        struct centerRParams
        {
            double area_min;
            double area_max;
            double aspect_ratio_max;
        };

        struct LocalRoiParams
        {
            double distance_ratio;
            float width;
        };

        RuneDetector(const ArrowParams &arrowP, const ArmorParams &armorP,
                     const centerRParams &centerRP, const LocalRoiParams &l, const EnemyColor detect_color, bool debug);
        bool detect(const cv::Mat &frame);

        cv::Mat m_imageArrow;  // 检测箭头用的二值化图片
        cv::Mat m_imageArmor;  // 检测装甲板边框用的二值化图片
        cv::Mat m_imageCenter; // 检测中心 R 用的二值化图片
        cv::Mat m_imageShow;   // 可视化图片

        // params
        ArrowParams arrow_params;
        ArmorParams armor_params;
        centerRParams centerR_params;
        LocalRoiParams local_roi_params;
        EnemyColor color;
        bool debug_;

        /**
         * @brief
         * 得到像素坐标系特征点，分别为装甲板内灯条的左上，右上，外灯条的中上，左下，右下，中心R。
         * @return std::vector<cv::Point2f>
         */
        inline std::vector<cv::Point2f> getCameraPoints()
        {
            return {m_armor.m_top, m_armor.m_right, m_armor.m_in, m_armor.m_left, m_centerR.m_center};
        }

    private:
        void preprocess(const cv::Mat &frame);
        bool detectArrow();
        bool detectArmor();
        bool detectCenterR();
        void setLocalRoi();
        void setArmor();
        void setGlobalRoi();
        void resetRoi(cv::Rect2f &rect, const cv::Mat &mat);
        void resetRoi(cv::Rect2f &rect, int rows, int cols);
        void resetRoi(cv::Rect2f &rect, const cv::Rect2f &lastRoi);
        double calAngleBetweenLightlines(const Lightline &l1, const Lightline &l2);
        bool findCenterR(CenterR &center, const std::vector<Lightline> &lightlines, const Arrow &arrow,
                         const Armor &armor);
        bool findArmor(Armor &armor, std::vector<ArmorContour> &armor_contours, const Arrow &arrow);
        bool findArmorContours(const cv::Mat &image, std::vector<ArmorContour> &contours, const cv::Rect2f &globalRoi,
                              const cv::Rect2f &localRoi);
        bool findCenterLightlines(const cv::Mat &image, std::vector<Lightline> &lightlines,
                                  const cv::Rect2f &globalRoi, const cv::Rect2f &localRoi);
        void findArrowLightlines(const cv::Mat &binary, std::vector<Lightline> &lightlines, const cv::Rect2f &roi);
        bool findArrow(Arrow &arrow, const std::vector<Lightline> &lightlines, const cv::Rect2f &roi);
        void draw(const Lightline &lightline, const cv::Scalar &color, const int thickness = 1,
                  const cv::Rect2f &localRoi = cv::Rect2f(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT));
        void draw(const cv::RotatedRect &rotatedRect, const cv::Scalar &color, const int thickness = 1,
                  const cv::Rect2f &localRoi = cv::Rect2f(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT));
        void draw(const cv::Rect2f &rect, const cv::Scalar &color, const int thickness = 1,
                  const cv::Rect2f &localRoi = cv::Rect2f(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT));
        void draw(const std::vector<cv::Point2f> &points, const cv::Scalar &color, const int thickness = 1,
                  const cv::Rect2f &localRoi = cv::Rect2f(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT));
        void draw(const cv::Point2f *points, const size_t size, const cv::Scalar &color, const int thickness = 1,
                  const cv::Rect2f &localRoi = cv::Rect2f(0, 0, IMAGE_WIDTH, IMAGE_HEIGHT));
        bool matchPoints(std::vector<PolarPoint> &points);
        bool matchPoints(std::vector<PolarPoint>& points, double& ratio);
        std::vector<std::vector<PolarPoint>> getCombinationsIterative(const std::vector<std::vector<PolarPoint>>& input);
        Arrow m_arrow;     // 箭头
        Armor m_armor;     // 装甲板
        CenterR m_centerR; // 中心 R

        cv::Mat m_localMask;    // 局部 roi 的掩码
        cv::Rect2f m_globalRoi; // 全局 roi ，用来圈定识别的范围，加快处理速度
        int m_lightArmorNum;
        cv::Rect2f m_armorRoi;  // 装甲板 roi
        cv::Rect2f m_centerRoi; // 中心 R roi
        Status m_status;        // 检测标志，包括成功、箭头检测失败、装甲板检测失败、中心 R 检测失败
    };
} // namespace imca::rune
