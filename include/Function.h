#pragma once

#include "d435i.h"
#include "UR5.h"
#include <opencv2/aruco.hpp>
#include <QObject>

struct MouseData
{
    vector<int> ids;
    vector<vector<Point2d>> image_point_4s;
    vector<vector<double>> target_pose_s;
    vector<double> init_pose;
    UR5 *p_ur5;
    Camera *p_camera;
    bool close;
};

// void shelf_grasp(Camera * camera, UR5 * ur5, cv::Mat matrix);
// void qr_grasp();
// void onMouse(int event, int x, int y, int flags, void* userdata);

class Function : public QObject
{
    Q_OBJECT
public:
    explicit Function(QObject *parent = nullptr);
    ~Function();
    // 计算特征点到相机的位姿
    bool calPoseByPnP(cv::Mat image, int featurePointsMode, cv::Mat & Tp2c);
    // 从图像中获取椭圆
    std::vector<cv::Point2d> get_ellipse(cv::Mat image); 
    // 从图像中获取圆
    std::vector<cv::Point2d> get_circle(cv::Mat image); 
    // ransac计算位姿
    tuple<cv::Mat, cv::Mat, vector<cv::Point2f>, string> ransac_calc_plane(int featurePointsMode, std::vector<cv::Point3d> object_points, std::vector<cv::Point2d> image_points);
    // 计算重投影误差
    tuple<cv::Mat, cv::Mat, vector<cv::Point2f>, int> get_projection_error(std::vector<cv::Point3d> object_points, std::vector<cv::Point2d> image_points);
    // 计算二维码到相机的位姿
    bool calPoseByQRCode(cv::Mat image, int QRCodeMode, cv::Mat & Tp2c);
    // 计算待抓取位姿与机械臂基座之间的相对位姿
    void calGraspPosCal(cv::Mat Tp2g, cv::Mat Tc2p, cv::Mat & Te2b, cv::Mat & Tc2e, cv::Mat & Tg2e, cv::Mat & Tg2b, cv::Mat & Tp2b);

signals:
    Q_INVOKABLE void shelf_grasp_finished();
    Q_INVOKABLE void init_grasp_finished();
    Q_INVOKABLE void init_pose_change(int index);

public slots:
    void shelf_grasp(Camera * camera, UR5 * ur5, cv::Mat matrix);
    // void shelf_grasp2(Camera * camera, UR5 * ur5, cv::Mat matrix);
    // void qr_grasp(Camera * camera, UR5 * ur5);
    // void onMouse(int event, int x, int y, int flags, void* userdata);
    // void setMouseData(MouseData * mouseData);
    void recv_mat(cv::Mat);
    void init_UR5_position(UR5 * ur5);
    void change_init_pose(int index);

private:
    cv::Mat current_mat;
    cv::Mat getRotationMatrixAroundAxis(double x, double y, double z, double angle);
    cv::Point2d rotatePoint(cv::Point2d p, cv::Point2d center, double theta);
    int test = 0;

    std::vector< std::vector<double> > init_poses = {
        {-0.0637563, -0.40863, 0.189758, -1.65466, -1.59605, 0.854768, }, // 斜看试管右侧
        {0.0514731, -0.44477, 0.469507, -0.0199394, -3.12947, 0.00660631}, // 高处水平向下看
        { 0.107076, -0.411211, 0.356728, -0.528978, 2.71514, -0.070417,}, // 斜着看左
        // { -0.111523, -0.416758, 0.335377, -0.00923894, -2.87088, -0.241265, }, // 斜着看右
        // {-0.0619756, -0.443327, 0.407511, -0.0199297, -3.12949, 0.00665348}, // 水平向下看
        // {0.0340289, -0.427769, 0.572794, -0.0199472, -3.12951, 0.00662037}, // 更高处水平向下看
    };

    // vector<double> init_pose = { 0.107076, -0.411211, 0.356728, -0.528978, 2.71514, -0.070417,}; // 斜着看左
    // vector<double> init_pose = { -0.111523, -0.416758, 0.335377, -0.00923894, -2.87088, -0.241265, }; // 斜着看右
    // vector<double> init_pose = {-0.0619756, -0.443327, 0.407511, -0.0199297, -3.12949, 0.00665348}; // 水平向下看
    // vector<double> init_pose = {0.0514731, -0.44477, 0.469507, -0.0199394, -3.12947, 0.00660631}; // 高处水平向下看
    // vector<double> init_pose = {0.0340289, -0.427769, 0.572794, -0.0199472, -3.12951, 0.00662037}; // 更高处水平向下看
    
    vector<double> init_pose = {0.107081, -0.411207, 0.356718, -0.574581, 2.9723, 0.000157606,};
};
