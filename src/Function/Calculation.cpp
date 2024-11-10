#include "Calculation.h"
#include <random>
#include <opencv2/aruco/charuco.hpp>


/**
 * @brief 计算特征点和相机直接的齐次坐标变换矩阵
 * 
 * @param image 输入图像
 * @param featurePointsMode 特征点模式: 1试管架上方，2试管架左侧，3试管架右侧 
 * @param Tp2c 试管架对应部分特征点与相机坐标系的变换矩阵
 * @return true 匹配成功
 * @return false 匹配失败，不是正确的试管架观测位置或无试管架
 */ 
bool Calculation::calPoseByPnP(cv::Mat image, int featurePointsMode, cv::Mat & Tp2c)
{
    std::vector<cv::Point3d> object_point; // 试管架上目标点的在试管架坐标系下的坐标
    switch (featurePointsMode)
    {
    case 1: // 试管架上方
    {       // 1         4
            // 2
            // 3         5
        object_point = {cv::Point3d(-0.025, -0.01, 0.00), cv::Point3d(-0.025, 0.00, 0.00), cv::Point3d(-0.025, 0.01, 0.00), cv::Point3d(0.025, -0.01, 0.00), cv::Point3d(0.025, 0.01, 0.00)};
    }
    break;
    case 2: // 试管架左侧
    {       //      3
            // 1         4
            // 2         5
        object_point = {cv::Point3d(-0.01, 0.00, 0.00), cv::Point3d(-0.01, 0.01, 0.00), cv::Point3d(0.00, -0.01, 0.00), cv::Point3d(0.01, 0.00, 0.00), cv::Point3d(0.01, 0.01, 0.00)};
    }break;
    case 3: // 试管架右侧
    {       //      2    5
            // 1    3
            //      4
        object_point = {cv::Point3d(-0.01, 0.00, 0.00), cv::Point3d(0.01, -0.01, 0.00), cv::Point3d(0.00, -0.01, 0.00), cv::Point3d(0.00, 0.00, 0.00), cv::Point3d(0.00, 0.01, 0.00)};
    }break;
    default:
        break;
    } 

    // 图像预处理 选出红色区域
    cv::Mat hsv_mat, mask_low, mask_high, mask;
    cv::Mat color_mat = image.clone();
    cv::medianBlur(color_mat, color_mat, 3); // 滤波
    cv::cvtColor(color_mat, hsv_mat, cv::COLOR_BGR2HSV); // 颜色空间转换
    cv::inRange(hsv_mat, cv::Scalar(0, 43, 46), cv::Scalar(10, 255, 255), mask_low); // 提取红色
    cv::inRange(hsv_mat, cv::Scalar(156, 43, 46), cv::Scalar(180, 255, 255), mask_high); // 提取红色
    cv::addWeighted(mask_low, 1, mask_high, 1, 0, mask); // 拼接掩模
    cv::Mat element1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)); // 
    cv::Mat element2 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2));
    cv::dilate(mask, mask, element1);

    // 获取图上的圆点
    std::vector<cv::Point2d> circles = get_ellipse(image);
    // 提取其中红色的圆点
    std::vector<cv::Point2d> red_points;
    cv::Point2d circle;
    for (size_t i = 0; i < circles.size(); i++) {
        circle = circles[i];
        // 放置数组越界
        if (circle.x < 0 || circle.y < 0 || circle.x > color_mat.cols || circle.y > color_mat.rows)
            continue; 
        // 该点原图区域为红色
        if (mask.at<uchar>(cvRound(circle.y), cvRound(circle.x)) == 255) {
            red_points.push_back(circle);
        }
    }

    if (red_points.size() < 5) { // 红色特征点小于5个，匹配失败
        return false;
    }

    // 获取特征点和相机之间的齐次变换矩阵
    auto shelf_pose = ransac_calc_plane(featurePointsMode, object_point, red_points);
    cv::Mat R = std::get<0>(shelf_pose);
    cv::Mat T = std::get<1>(shelf_pose);
    std::string type = std::get<3>(shelf_pose);
    if (type == "none") {
        return false;
    }
    else {
        Tp2c = cv::Mat::eye(4, 4, CV_64F);
        R.copyTo(Tp2c(cv::Rect(0, 0, 3, 3)));
        T.copyTo(Tp2c(cv::Rect(3, 0, 1, 3)));
        return true;
    }
}

/**
 * @brief 用于从图像中提取椭圆
 * 
 * @param image 输入图像
 * @return std::vector<cv::Point2d> 图像上所有的椭圆构成的列表
 */
std::vector<cv::Point2d> Calculation::get_ellipse(cv::Mat image)
{
    std::vector<cv::Point2d> ellipses;
    cv::Mat resized_mat, resized_gray, edges;
    double scale = 3.0;
    cv::resize(image, resized_mat, cv::Size(image.cols * scale, image.rows * scale), 0, 0, cv::INTER_CUBIC);
    cv::cvtColor(resized_mat, resized_gray, cv::COLOR_BGR2GRAY);
    GaussianBlur(resized_gray, resized_gray, cv::Size(9, 9), 2, 2);
    cv::Canny(resized_gray, edges, 70, 200, 5);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(edges, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
    
    std::vector<cv::RotatedRect> minEllipse(contours.size());
    for (int i = 0; i < contours.size(); i++)
    {
        if (contours[i].size() > 5)
        {
            minEllipse[i] = cv::fitEllipse(contours[i]);
            cv::ellipse(resized_mat, minEllipse[i], cv::Scalar(0, 255, 0), 2);
            cv::Point2d center = minEllipse[i].center;
            ellipses.push_back(center);
        }
    }
    // 椭圆参数缩放为原始尺寸
    for (size_t i = 0; i < ellipses.size(); i++)
    {
        ellipses[i].x /= scale;
        ellipses[i].y /= scale;
    }

    return ellipses;
}

/**
 * @brief 通过随机抽样查找符合特点排列顺序的5个特征点，通过重投影误差判断匹配准确性
 * 
 * @param featurePointsMode 特征点模式: 1试管架上方，2试管架左侧，3试管架右侧 
 * @param object_point 试管架上特征点的物理坐标
 * @param image_point 包含特征点的所有点
 * @return tuple<cv::Mat, cv::Mat, vector<cv::Point2f>, string> 试管架和相机之间的R、T变换矩阵、特征点的重投影坐标、特征点的朝向说明
 */
std::tuple<cv::Mat, cv::Mat, std::vector<cv::Point2f>, std::string> Calculation::ransac_calc_plane(int featurePointsMode, std::vector<cv::Point3d> object_point, std::vector<cv::Point2d> image_point)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::tuple<cv::Mat, cv::Mat, std::vector<cv::Point2f>> shelf_pose;
    double min_x, max_x, min_y, max_y;
    std::vector<cv::Point2d> center_points_5, center_points_5_copy, center_points_5_sequence;

    for (size_t iteration = 0; iteration < 10000; iteration++)
    {
        do // 随机取五个两两最大距离不超过 180 100 的点
        {
            center_points_5.clear();
            min_x = 1000;
            min_y = 1000;
            max_x = -1;
            max_y = -1;
            shuffle(image_point.begin(), image_point.end(), gen);
            for (size_t j = 0; j < 5; j++)
            {
                center_points_5.push_back(image_point[j]);
            }
            for (auto point : center_points_5)
            {
                min_x = std::min(min_x, point.x);
                min_y = std::min(min_y, point.y);
                max_x = std::max(max_x, point.x);
                max_y = std::max(max_y, point.y);
            }
        } while (max_x - min_x > 180 || max_y - min_y > 100);
        
        switch (featurePointsMode)
        {
        case 1: // 试管架上方的点
        {
            // condition 1: 左3 右2 upN ---------------------------------------------------------------------
            center_points_5_sequence.clear();
            std::sort(center_points_5.begin(), center_points_5.end(), [](const cv::Point2d& p1, const cv::Point2d& p2){
                return p1.x < p2.x;
            });
            center_points_5_copy = center_points_5;
            for (size_t i = 0; i < 3; i++)
            {
                auto minPoint = min_element(center_points_5_copy.begin(), 
                    center_points_5_copy.begin() + 3 - i,
                    [](const cv::Point2d& p1, const cv::Point2d& p2) {
                    return p1.y < p2.y;
                });
                center_points_5_sequence.push_back(*minPoint);
                center_points_5_copy.erase(minPoint);
            }
            for(size_t i = 0; i < 2; i++)
            {
                auto minPoint = min_element(center_points_5_copy.begin(), 
                    center_points_5_copy.end(),
                    [](const cv::Point2d& p1, const cv::Point2d& p2) {
                    return p1.y < p2.y;
                });
                center_points_5_sequence.push_back(*minPoint);
                center_points_5_copy.erase(minPoint);
            }
            auto pose = get_projection_error(object_point, center_points_5_sequence);
            double projection_error = std::get<3>(pose);
            if (projection_error < 2)
            {
                return std::make_tuple(std::get<0>(pose), std::get<1>(pose), std::get<2>(pose), "upN");
            }            

            // condition 2: 左2 右3 upP ---------------------------------------------------------------------
            center_points_5_sequence.clear();
            std::sort(center_points_5.begin(), center_points_5.end(), [](const cv::Point2d& p1, const cv::Point2d& p2){
                return p1.x > p2.x;
            });
            center_points_5_copy = center_points_5;
            for(size_t i = 0; i < 3; i++)
            {
                auto minPoint = min_element(center_points_5_copy.begin(), 
                    center_points_5_copy.begin() + 3 - i,
                    [](const cv::Point2d& p1, const cv::Point2d& p2) {
                    return p1.y > p2.y;
                });
                center_points_5_sequence.push_back(*minPoint);
                center_points_5_copy.erase(minPoint);
            }
            for(size_t i = 0; i < 2; i++)
            {
                auto minPoint = min_element(center_points_5_copy.begin(), 
                    center_points_5_copy.end(),
                    [](const cv::Point2d& p1, const cv::Point2d& p2) {
                    return p1.y > p2.y;
                });
                center_points_5_sequence.push_back(*minPoint);
                center_points_5_copy.erase(minPoint);
            }
            pose = get_projection_error(object_point, center_points_5_sequence);
            projection_error = std::get<3>(pose);
            if (projection_error < 2)
            {
                return std::make_tuple(std::get<0>(pose), std::get<1>(pose), std::get<2>(pose), "upP");
            }

            // condition 3: 上3 下2 upL ---------------------------------------------------------------------
            center_points_5_sequence.clear();
            std::sort(center_points_5.begin(), center_points_5.end(), [](const cv::Point2d& p1, const cv::Point2d& p2){
                return p1.y < p2.y;
            });
            center_points_5_copy = center_points_5;
            for(size_t i = 0; i < 3; i++)
            {
                auto minPoint = min_element(center_points_5_copy.begin(), 
                    center_points_5_copy.begin() + 3 - i,
                    [](const cv::Point2d& p1, const cv::Point2d& p2) {
                    return p1.x > p2.x;
                });
                center_points_5_sequence.push_back(*minPoint);
                center_points_5_copy.erase(minPoint);
            }
            for (size_t i = 0; i < 2; i++)
            {
                auto minPoint = min_element(center_points_5_copy.begin(), 
                    center_points_5_copy.end(),
                    [](const cv::Point2d& p1, const cv::Point2d& p2) {
                    return p1.x > p2.x;
                });
                center_points_5_sequence.push_back(*minPoint);
                center_points_5_copy.erase(minPoint);
            }
            pose = get_projection_error(object_point, center_points_5_sequence);
            projection_error = std::get<3>(pose);
            if (projection_error < 2)
            {
                return std::make_tuple(std::get<0>(pose), std::get<1>(pose), std::get<2>(pose), "upL");
            }

            // condition 4: 上2 下3 upR ---------------------------------------------------------------------
            center_points_5_sequence.clear();
            std::sort(center_points_5.begin(), center_points_5.end(), [](const cv::Point2d& p1, const cv::Point2d& p2){
                return p1.y > p2.y;
            });
            center_points_5_copy = center_points_5;
            for(size_t i = 0; i < 3; i++)
            {
                auto minPoint = min_element(center_points_5_copy.begin(), 
                    center_points_5_copy.begin() + 3 - i,
                    [](const cv::Point2d& p1, const cv::Point2d& p2) {
                    return p1.x < p2.x;
                });
                center_points_5_sequence.push_back(*minPoint);
                center_points_5_copy.erase(minPoint);
            }
            for(size_t i = 0; i < 2; i++)
            {
                auto minPoint = min_element(center_points_5_copy.begin(), 
                    center_points_5_copy.end(),
                    [](const cv::Point2d& p1, const cv::Point2d& p2) {
                    return p1.x < p2.x;
                });
                center_points_5_sequence.push_back(*minPoint);
                center_points_5_copy.erase(minPoint);
            }
            pose = get_projection_error(object_point, center_points_5_sequence);
            projection_error = std::get<3>(pose);
            if (projection_error < 2)
            {
                return std::make_tuple(std::get<0>(pose), std::get<1>(pose), std::get<2>(pose), "upR");
            }
        }break;
        case 2: // 试管架左侧的点
        {
            // condition 1 上1 下4 leftP
            center_points_5_sequence.clear();
            std::sort(center_points_5.begin(), center_points_5.end(), [](const cv::Point2d& p1, const cv::Point2d& p2){
                return p1.x < p2.x;
            });
            center_points_5_copy = center_points_5;
            for(size_t i = 0; i < 2; i++)
            {
                auto minPoint = min_element(center_points_5_copy.begin(), 
                    center_points_5_copy.begin() + 2 - i,
                    [](const cv::Point2d& p1, const cv::Point2d& p2) {
                    return p1.y < p2.y;
                });
                center_points_5_sequence.push_back(*minPoint);
                center_points_5_copy.erase(minPoint);
            }
            center_points_5_sequence.push_back(center_points_5_copy[0]);
            center_points_5_copy.erase(center_points_5_copy.begin());
            for(size_t i = 0; i < 2; i++)
            {
                auto minPoint = min_element(center_points_5_copy.begin(), 
                    center_points_5_copy.begin() + 2 - i,
                    [](const cv::Point2d& p1, const cv::Point2d& p2) {
                    return p1.y < p2.y;
                });
                center_points_5_sequence.push_back(*minPoint);
                center_points_5_copy.erase(minPoint);
            }
            auto pose = get_projection_error(object_point, center_points_5_sequence);
            double projection_error = std::get<3>(pose);
            if (projection_error < 2)
            {
                return std::make_tuple(std::get<0>(pose), std::get<1>(pose), std::get<2>(pose), "leftP");
            }

            // condition 2 上4 下1 leftN
            center_points_5_sequence.clear();
            std::sort(center_points_5.begin(), center_points_5.end(), [](const cv::Point2d& p1, const cv::Point2d& p2){
                return p1.x > p2.x;
            });
            center_points_5_copy = center_points_5;
            for(size_t i = 0; i < 2; i++)
            {
                auto minPoint = min_element(center_points_5_copy.begin(), 
                    center_points_5_copy.begin() + 2 - i,
                    [](const cv::Point2d& p1, const cv::Point2d& p2) {
                    return p1.y > p2.y;
                });
                center_points_5_sequence.push_back(*minPoint);
                center_points_5_copy.erase(minPoint);
            }
            center_points_5_sequence.push_back(center_points_5_copy[0]);
            center_points_5_copy.erase(center_points_5_copy.begin());
            for(size_t i = 0; i < 2; i++)
            {
                auto minPoint = min_element(center_points_5_copy.begin(), 
                    center_points_5_copy.begin() + 2 - i,
                    [](const cv::Point2d& p1, const cv::Point2d& p2) {
                    return p1.y > p2.y;
                });
                center_points_5_sequence.push_back(*minPoint);
                center_points_5_copy.erase(minPoint);
            }
            pose = get_projection_error(object_point, center_points_5_sequence);
            projection_error = std::get<3>(pose);
            if (projection_error < 2)
            {
                return std::make_tuple(std::get<0>(pose), std::get<1>(pose), std::get<2>(pose), "leftN");
            }

            // condition 3 左1 右4 leftL
            center_points_5_sequence.clear();
            std::sort(center_points_5.begin(), center_points_5.end(), [](const cv::Point2d& p1, const cv::Point2d& p2){
                return p1.y > p2.y;
            });
            center_points_5_copy = center_points_5;
            for(size_t i = 0; i < 2; i++)
            {
                auto minPoint = min_element(center_points_5_copy.begin(), 
                    center_points_5_copy.begin() + 2 - i,
                    [](const cv::Point2d& p1, const cv::Point2d& p2) {
                    return p1.x < p2.x;
                });
                center_points_5_sequence.push_back(*minPoint);
                center_points_5_copy.erase(minPoint);
            }
            center_points_5_sequence.push_back(center_points_5_copy[0]);
            center_points_5_copy.erase(center_points_5_copy.begin());
            for(size_t i = 0; i < 2; i++)
            {
                auto minPoint = min_element(center_points_5_copy.begin(), 
                    center_points_5_copy.begin() + 2 - i,
                    [](const cv::Point2d& p1, const cv::Point2d& p2) {
                    return p1.x < p2.x;
                });
                center_points_5_sequence.push_back(*minPoint);
                center_points_5_copy.erase(minPoint);
            }
            pose = get_projection_error(object_point, center_points_5_sequence);
            projection_error = std::get<3>(pose);
            if (projection_error < 2)
            {
                return std::make_tuple(std::get<0>(pose), std::get<1>(pose), std::get<2>(pose), "leftL");
            }

            // condition 4 左4 右1 leftR
            center_points_5_sequence.clear();
            std::sort(center_points_5.begin(), center_points_5.end(), [](const cv::Point2d& p1, const cv::Point2d& p2){
                return p1.y < p2.y;
            });
            center_points_5_copy = center_points_5;
            for(size_t i = 0; i < 2; i++)
            {
                auto minPoint = min_element(center_points_5_copy.begin(), 
                    center_points_5_copy.begin() + 2 - i,
                    [](const cv::Point2d& p1, const cv::Point2d& p2) {
                    return p1.x > p2.x;
                });
                center_points_5_sequence.push_back(*minPoint);
                center_points_5_copy.erase(minPoint);
            }
            center_points_5_sequence.push_back(center_points_5_copy[0]);
            center_points_5_copy.erase(center_points_5_copy.begin());
            for(size_t i = 0; i < 2; i++)
            {
                auto minPoint = min_element(center_points_5_copy.begin(), 
                    center_points_5_copy.begin() + 2 - i,
                    [](const cv::Point2d& p1, const cv::Point2d& p2) {
                    return p1.x > p2.x;
                });
                center_points_5_sequence.push_back(*minPoint);
                center_points_5_copy.erase(minPoint);
            }
            pose = get_projection_error(object_point, center_points_5_sequence);
            projection_error = std::get<3>(pose);
            if (projection_error < 2)
            {
                return std::make_tuple(std::get<0>(pose), std::get<1>(pose), std::get<2>(pose), "leftR");
            }
        }break;
        case 3: // 试管架右侧的点
        {
            // condition 1 左中1 中3 右上1 rightP
            center_points_5_sequence.clear();
            std::sort(center_points_5.begin(), center_points_5.end(), [](const cv::Point2d& p1, const cv::Point2d& p2){
                return p1.x < p2.x;
            });
            center_points_5_copy = center_points_5;
            center_points_5_sequence.push_back(center_points_5_copy[0]);
            center_points_5_copy.erase(center_points_5_copy.begin());
            for (size_t i = 0; i < 3; i++)
            {
                auto minPoint = min_element(center_points_5_copy.begin(), 
                    center_points_5_copy.begin() + 3 - i,
                    [](const cv::Point2d& p1, const cv::Point2d& p2) {
                    return p1.y < p2.y;
                });
                center_points_5_sequence.push_back(*minPoint);
                center_points_5_copy.erase(minPoint);
            }
            center_points_5_sequence.push_back(center_points_5_copy[0]);
            center_points_5_copy.erase(center_points_5_copy.begin());
            auto pose = get_projection_error(object_point, center_points_5_sequence);
            double projection_error = std::get<3>(pose);
            if (projection_error < 2)
            {
                return std::make_tuple(std::get<0>(pose), std::get<1>(pose), std::get<2>(pose), "rightP");
            }

            // condition 2 左下1 中3 右中1 rightN
            center_points_5_sequence.clear();
            std::sort(center_points_5.begin(), center_points_5.end(), [](const cv::Point2d& p1, const cv::Point2d& p2){
                return p1.x > p2.x;
            });
            center_points_5_copy = center_points_5;
            center_points_5_sequence.push_back(center_points_5_copy[0]);
            center_points_5_copy.erase(center_points_5_copy.begin());
            for (size_t i = 0; i < 3; i++)
            {
                auto minPoint = min_element(center_points_5_copy.begin(), 
                    center_points_5_copy.begin() + 3 - i,
                    [](const cv::Point2d& p1, const cv::Point2d& p2) {
                    return p1.y > p2.y;
                });
                center_points_5_sequence.push_back(*minPoint);
                center_points_5_copy.erase(minPoint);
            }
            center_points_5_sequence.push_back(center_points_5_copy[0]);
            center_points_5_copy.erase(center_points_5_copy.begin());
            pose = get_projection_error(object_point, center_points_5_sequence);
            projection_error = std::get<3>(pose);
            if (projection_error < 2)
            {
                return std::make_tuple(std::get<0>(pose), std::get<1>(pose), std::get<2>(pose), "rightN");
            }

            // condition 3 上左1 中3 下中1 rightL
            center_points_5_sequence.clear();
            std::sort(center_points_5.begin(), center_points_5.end(), [](const cv::Point2d& p1, const cv::Point2d& p2){
                return p1.y > p2.y;
            });
            center_points_5_copy = center_points_5;
            center_points_5_sequence.push_back(center_points_5_copy[0]);
            center_points_5_copy.erase(center_points_5_copy.begin());
            for (size_t i = 0; i < 3; i++)
            {
                auto minPoint = min_element(center_points_5_copy.begin(), 
                    center_points_5_copy.begin() + 3 - i,
                    [](const cv::Point2d& p1, const cv::Point2d& p2) {
                    return p1.x < p2.x;
                });
                center_points_5_sequence.push_back(*minPoint);
                center_points_5_copy.erase(minPoint);
            }
            center_points_5_sequence.push_back(center_points_5_copy[0]);
            center_points_5_copy.erase(center_points_5_copy.begin());
            pose = get_projection_error(object_point, center_points_5_sequence);
            projection_error = std::get<3>(pose);
            if (projection_error < 2)
            {
                return std::make_tuple(std::get<0>(pose), std::get<1>(pose), std::get<2>(pose), "rightL");
            }

            // condition 4 上中1 中3 下右1 rightR
            center_points_5_sequence.clear();
            std::sort(center_points_5.begin(), center_points_5.end(), [](const cv::Point2d& p1, const cv::Point2d& p2){
                return p1.y < p2.y;
            });
            center_points_5_copy = center_points_5;
            center_points_5_sequence.push_back(center_points_5_copy[0]);
            center_points_5_copy.erase(center_points_5_copy.begin());
            for (size_t i = 0; i < 3; i++)
            {
                auto minPoint = min_element(center_points_5_copy.begin(), 
                    center_points_5_copy.begin() + 3 - i,
                    [](const cv::Point2d& p1, const cv::Point2d& p2) {
                    return p1.x > p2.x;
                });
                center_points_5_sequence.push_back(*minPoint);
                center_points_5_copy.erase(minPoint);
            }
            center_points_5_sequence.push_back(center_points_5_copy[0]);
            center_points_5_copy.erase(center_points_5_copy.begin());
            pose = get_projection_error(object_point, center_points_5_sequence);
            projection_error = std::get<3>(pose);
            if (projection_error < 2)
            {
                return std::make_tuple(std::get<0>(pose), std::get<1>(pose), std::get<2>(pose), "rightR");
            }
        }break;
        default:
            break;
        }

    }
    return std::make_tuple(cv::Mat(), cv::Mat(), std::vector<cv::Point2f>(), "none");
}

/**
 * @brief 调用SovlePnP计算变换矩阵并计算重投影误差大小
 * 
 * @param object_points 试管架上特征点的物理坐标
 * @param image_points 5个特征点
 * @return tuple<cv::Mat, cv::Mat, vector<cv::Point2f>> 试管架和相机之间的R、T变换矩阵、特征点的重投影坐标
 */
std::tuple<cv::Mat, cv::Mat, std::vector<cv::Point2f>, int> Calculation::get_projection_error(std::vector<cv::Point3d> object_points, std::vector<cv::Point2d> image_points)
{
    double projection_error = 0;
    std::vector<cv::Point3f> object_points_3f;
    for (const auto& point3d : object_points)
    {
        cv::Point3f point3f = cv::Point3f(point3d);
        object_points_3f.push_back(point3f);
    }

    // 读取相机内参:内参矩阵、畸变向量
    cv::Mat intrinsics_mat, distCoeffs;
    cv::FileStorage fs_read("../configs/config.yml", cv::FileStorage::READ);
    if (fs_read.isOpened()) {
        fs_read["Intrinsics_Mat"] >> intrinsics_mat;
        fs_read["DistCoeffs_Mat"] >> distCoeffs;
        fs_read.release();
    }

    cv::Mat V_Shelf2Cam, T_Shelf2Cam, R_Shelf2Cam;
    solvePnP(object_points, image_points, intrinsics_mat, distCoeffs, V_Shelf2Cam, T_Shelf2Cam);
    Rodrigues(V_Shelf2Cam, R_Shelf2Cam);
    std::vector<cv::Point2f> projection_points;
    projectPoints(object_points_3f, V_Shelf2Cam, T_Shelf2Cam, intrinsics_mat, distCoeffs, projection_points);

    for (size_t i = 0; i < image_points.size(); i++)
    {
        double dx = image_points[i].x - projection_points[i].x;
        double dy = image_points[i].y - projection_points[i].y;
        projection_error += sqrt(dx * dx + dy * dy);
    }
    return make_tuple(R_Shelf2Cam.clone(), T_Shelf2Cam.clone(), projection_points, projection_error);
}

/**
 * @brief 在图像上检测二维码并根据计算指定id的二维码和相机之间的齐次变换矩阵
 * 
 * @param image 输入图像
 * @param QRCodeMode 二维码编号
 * @param Tp2c 齐次变换矩阵
 * @return true 找到了指定的二维码并计算出了齐次变换矩阵
 * @return false 未找到指定的二维码或计算失败
 */
bool Calculation::calPoseByQRCode(cv::Mat image, int QRCodeMode, cv::Mat & Tp2c)
{
    // 读取相机内参:内参矩阵、畸变向量
    cv::Mat intrinsics_mat, distCoeffs;
    cv::FileStorage fs_read("../configs/config.yml", cv::FileStorage::READ);
    if (fs_read.isOpened()) {
        fs_read["Intrinsics_Mat"] >> intrinsics_mat;
        fs_read["DistCoeffs_Mat"] >> distCoeffs;
        fs_read.release();
    }

    const double qrSize = 0.070; // QR码的尺寸 7cm

    // 二维码四个角点物理坐标
    // 1        2
    // 4        3
    std::vector<cv::Point3d> object_point = {
        cv::Point3d(-qrSize / 2, -qrSize / 2, 0.00), // 左上 1
        cv::Point3d(qrSize / 2, -qrSize / 2, 0.00), // 右上 2
        cv::Point3d(qrSize / 2, qrSize / 2, 0.00), // 右下 3
        cv::Point3d(-qrSize / 2, qrSize / 2, 0.00), // 左下 4
    };

    // 扫描二维码
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250); // 二维码字典
    std::vector<int> markerIds; // 扫描到的所有二维码的编号
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates; // 扫描到的所有二维码的四个角点坐标
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters(); // 二维码检测参数
    cv::aruco::ArucoDetector detector(dictionary, detectorParams); // 二维码检测器
    detector.detectMarkers(image, markerCorners, markerIds, rejectedCandidates); // 检测二维码

    if (markerIds.size() == 0) {
        return false;
    }
    else {
        // 找到指定的二维码
        for (size_t i = 0; i < markerIds.size(); i++) {
            if (markerIds[i] == QRCodeMode) {
                // 计算二维码和相机之间的齐次变换矩阵
                cv::Mat rvec, tvec;
                cv::aruco::estimatePoseSingleMarkers(markerCorners, qrSize, intrinsics_mat, distCoeffs, rvec, tvec);
                cv::Mat R;
                cv::Rodrigues(rvec, R);
                Tp2c = cv::Mat::eye(4, 4, CV_64F);
                R.copyTo(Tp2c(cv::Rect(0, 0, 3, 3)));
                tvec.copyTo(Tp2c(cv::Rect(3, 0, 1, 3)));
                return true;
            }
        }
        // 未找到指定的二维码
        return false;
    }
    return false;
}

/**
 * @brief 计算待抓取位姿与机械臂基座之间的相对位姿
 * 
 * @param Tp2g 特征点与夹爪之间的相对位姿，该由任务规划软件决定 输入
 * @param Tc2p 相机与特征点之间的相对位姿，根据图像进行计算 输入
 * @param Te2b 机械臂末端与基座之间的相对位姿，根据机械臂控制器获得 输入
 * @param Tc2e 相机与机械臂末端之间的相对位姿关系，标定而来 读取本地 (ps:标定的结果实际上是相机与夹爪末端之间的相对位姿关系)
 * @param Tg2e 夹爪与机械臂末端之间的相对位姿关系，标定而来 读取本地
 * @param Tg2b 夹爪与机械臂基座之间的相对位姿关系，为计算结果 输出
 */
void Calculation::calGraspPosCal(cv::Mat Tp2g, cv::Mat Tc2p, cv::Mat & Te2b, cv::Mat & Tc2e, cv::Mat & Tg2e, cv::Mat & Tg2b)
{
    if (Tp2g.empty() || Tc2p.empty() || Tg2b.empty()) {
        std::cout << "Input matrix is empty, can not calculate!" << std::endl;
        return;
    }

    // 读取相机与夹爪末端之间的相对位姿关系 Tc2e
    cv::FileStorage fs_read("../configs/config.yml", cv::FileStorage::READ);
    cv::Mat R_Cam2End, T_Cam2End;
    if (fs_read.isOpened()) {
        fs_read["R_Cam2End"] >> R_Cam2End;
        fs_read["T_Cam2End"] >> T_Cam2End;
        fs_read.release();
    }
    Tc2e = cv::Mat::eye(4, 4, CV_64F);
    R_Cam2End.copyTo(Tc2e(cv::Rect(0, 0, 3, 3)));
    T_Cam2End.copyTo(Tc2e(cv::Rect(3, 0, 1, 3)));

    // 读取夹爪与机械臂末端之间的相对位姿关系 Tg2e
    cv::FileStorage fs_read2("../configs/config.yml", cv::FileStorage::READ);
    cv::Mat R_Grasp2End, T_Grasp2End;
    if (fs_read2.isOpened()) {
        fs_read2["R_Grasp2End"] >> R_Grasp2End;
        fs_read2["T_Grasp2End"] >> T_Grasp2End;
        fs_read2.release();
    }
    Tg2e = cv::Mat::eye(4, 4, CV_64F);
    R_Grasp2End.copyTo(Tg2e(cv::Rect(0, 0, 3, 3)));
    T_Grasp2End.copyTo(Tg2e(cv::Rect(3, 0, 1, 3)));

    // 计算特征点与机械臂基座之间的相对位姿关系 Tp2b

    // Tp2g 特征点与目标点时夹爪位置之间的相对位姿关系
    // Tc2p 相机与特征点之间的相对位姿关系
    // Te2b 机械臂末端与基座之间的相对位姿关系 (网络通信读取)
    // Tc2e 相机与机械臂末端之间的相对位姿关系
    // Tg2e 夹爪与机械臂末端之间的相对位姿关系
    // Tg2b 目标点时夹爪位置与机械臂基座之间的相对位姿关系

    // 目标点 P0 (物体坐标系)
    // 特征点 PX (物体坐标系)
    // P0 = Tp2g * PX
    // P1（基座坐标系）= Te2b *  Tc2e * Tc2p.inv() * P0
    Tg2b = Te2b *  Tc2e * Tc2p.inv() * Tp2g;
}
