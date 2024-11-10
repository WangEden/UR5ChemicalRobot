#ifndef __CALC__H__
#define __CALC__H__

#include <opencv2/aruco.hpp>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

class Calculation
{
    Calculation();
    ~Calculation();
    // 计算特征点到相机的位姿
    bool calPoseByPnP(cv::Mat image, int featurePointsMode, cv::Mat & Tp2c);
    // 从图像中获取椭圆
    std::vector<cv::Point2d> get_ellipse(cv::Mat image); 
    // 从图像中获取圆
    std::vector<cv::Point2d> get_circle(cv::Mat image); 
    // ransac计算位姿
    std::tuple<cv::Mat, cv::Mat, std::vector<cv::Point2f>, std::string> ransac_calc_plane(int featurePointsMode, std::vector<cv::Point3d> object_points, std::vector<cv::Point2d> image_points);
    // 计算重投影误差
    std::tuple<cv::Mat, cv::Mat, std::vector<cv::Point2f>, int> get_projection_error(std::vector<cv::Point3d> object_points, std::vector<cv::Point2d> image_points);
    // 计算二维码到相机的位姿
    bool calPoseByQRCode(cv::Mat image, int QRCodeMode, cv::Mat & Tp2c);
    // 计算待抓取位姿与机械臂基座之间的相对位姿
    void calGraspPosCal(cv::Mat Tp2g, cv::Mat Tc2p, cv::Mat & Te2b, cv::Mat & Tc2e, cv::Mat & Tg2e, cv::Mat & Tg2b);
};

#endif