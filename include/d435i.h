#ifndef d435i_H
#define d435i_H

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <tuple>
#include <vector>

using namespace std;

class Camera
{
private:
    int height = 480;
    int width = 640;
    int fps = 30;
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2_intrinsics color_intrinsics;
    rs2_intrinsics depth_intrinsics;
    cv::Mat intrinsics_mat, distCoeffs;
    float depth_scale;
    float depth_offset;
    float depth_fov_x;
    float depth_fov_y;
    rs2::align align_to_color;
    rs2::frameset frameset;
    cv::Mat color_mat;
    cv::Mat depth_mat;
    int mat_width = 0;
    int mat_height = 0;
    int count = 0;
    string color_filename;
    string depth_filename;
    // calibration
    cv::Mat gray_mat;
    vector<cv::Point2f> corners_2f;
    vector<cv::Point2f> corners_projection_2f;
    vector<cv::Point3f> corners_3f;
    vector<cv::Point2d> corners_2d;
    vector<cv::Point3d> corners_3d;
    cv::Mat V_Board2Cam, R_Board2Cam, T_Board2Cam;
    cv::Size chessboard_size;
    double square_size = 0.015;
    bool pattern_found;
    // shelf grasp
    double projection_error;
    vector<cv::Vec3f> circles;
    vector<cv::Point3f> object_points_3f;
    vector<cv::Point2d> centers, ellipses, image_points, center_points_copy, center_points_5;
    vector<cv::Point2f> projection_points;
    cv::Vec3i circle;
    cv::Point center;
    int radius;
    cv::Mat V_Shelf2Cam, R_Shelf2Cam, T_Shelf2Cam;
    cv::Mat V_QR2Cam, R_QR2Cam, T_QR2Cam;
    map<string, cv::FileNode> yamlData;
    // *************************************************
    rs2::sensor depth_sensor;
public:
    Camera();
    tuple<cv::Mat, cv::Mat> connect();
    void disconnect();
    tuple<cv::Mat, cv::Mat> get_images();
    int get_img_count();
    void save_images();
    void save_images(string color_name, string depth_name);
    void get_corners(bool save = false);
    tuple<cv::Mat, cv::Mat> get_R_board2cam(bool projection); 
    vector<cv::Vec3f> get_circles(bool save, int mode);
    vector<cv::Point2d> get_ellipse(bool save, int mode);
    tuple<cv::Mat, cv::Mat, vector<cv::Point2f>, string> get_shelf_pose(vector<vector<cv::Point3d>> object_points, vector<cv::Point2d> center_points, bool & pose_flag);
    tuple<cv::Mat, cv::Mat, vector<cv::Point2f>> get_projection_error(vector<cv::Point3d> object_points, vector<cv::Point2d> image_points);
    tuple<cv::Mat, cv::Mat, vector<cv::Point2f>> get_QR_pose(vector<cv::Point3d> object_points, vector<cv::Point2d> image_points);
    double getDistance(int x, int y);
};

double calculateMinDistance(const std::vector<cv::Point2d>& points);

#endif