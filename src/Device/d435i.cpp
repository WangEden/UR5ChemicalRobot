#include "d435i.h"
#include <opencv2/core/persistence.hpp>
#include <thread>
#include <chrono>
#include <thread>
#include <chrono>
#include <cstring>
// #include <cassert>
#include <algorithm>
#include <random>

using namespace std;

Camera::Camera() : align_to_color(RS2_STREAM_COLOR), chessboard_size(cv::Size(11, 8))
{
    // 读取存储的图像数量
    string path = "../data/ImageCount.yml";
    cv::FileStorage fs_read(path, cv::FileStorage::READ);
    if (fs_read.isOpened()) {
        fs_read["count"] >> this->count;
        // fs_read["R_Cam2End"] >> this->yamlData["R_Cam2End"];
        // fs_read["T_Cam2End"] >> this->yamlData["T_Cam2End"];
        fs_read.release();
    }

    for (int i = 0; i < this->chessboard_size.height; i++)
    {
        for (int j = 0; j < this->chessboard_size.width; j++)
        {
            this->corners_3d.push_back(cv::Point3d(j * this->square_size, i * this->square_size, 0.0));
            this->corners_3f.push_back(cv::Point3f(j * this->square_size, i * this->square_size, 0.0));
            // cout << "x:" << j * this->square_size << "\t" << "y:" << i * this->square_size << endl;
        }
    }
}

tuple<cv::Mat, cv::Mat> Camera::connect() {
    this->cfg.enable_stream(RS2_STREAM_COLOR, this->width, this->height, RS2_FORMAT_BGR8, this->fps);
    this->cfg.enable_stream(RS2_STREAM_DEPTH, this->width, this->height, RS2_FORMAT_Z16, this->fps);
    rs2::pipeline_profile profile = this->pipe.start(cfg);

    // 休眠短暂时间，确保深度帧可用
    std::this_thread::sleep_for(std::chrono::seconds(1));
    this->depth_sensor = this->pipe.get_active_profile().get_device().first<rs2::depth_sensor>();

    auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    this->depth_intrinsics = depth_stream.get_intrinsics();
    this->color_intrinsics = color_stream.get_intrinsics();

    this->intrinsics_mat = (cv::Mat_<double>(3, 3) << \
    this->color_intrinsics.fx, 0.0, this->color_intrinsics.ppx, \
    0.0, this->color_intrinsics.fy, this->color_intrinsics.ppy, \
    0.0, 0.0, 1.0);
    this->distCoeffs = (cv::Mat_<double>(5, 1) << this->color_intrinsics.coeffs[0], this->color_intrinsics.coeffs[1], \
    this->color_intrinsics.coeffs[2], this->color_intrinsics.coeffs[3], this->color_intrinsics.coeffs[4]);
    
    this->depth_fov_x = (atan(depth_intrinsics.ppx / depth_intrinsics.fx) + atan((depth_intrinsics.width - depth_intrinsics.ppx) / depth_intrinsics.fx)) * 180.0f / M_PI;
    this->depth_fov_y = (atan(depth_intrinsics.ppy / depth_intrinsics.fy) + atan((depth_intrinsics.height - depth_intrinsics.ppy) / depth_intrinsics.fy)) * 180.0f / M_PI;
    
    // cout << this->color_intrinsics.width << this->color_intrinsics.height << endl;
    // cout << "intri: " << this->intrinsics_mat << endl;
    // for (size_t i = 0; i < 5; i++)
    // {
    //     cout << this->distCoeffs[i] << " " << endl;
    // }

    this->depth_scale = profile.get_device().first<rs2::depth_sensor>().get_depth_scale();
    this->depth_offset = profile.get_device().first<rs2::depth_sensor>().get_option(RS2_OPTION_DEPTH_UNITS);

    cout << "Camera connected" << endl;
    this_thread::sleep_for(chrono::milliseconds(2000));

    return make_tuple(this->intrinsics_mat.clone(), this->distCoeffs.clone());
}

void Camera::disconnect() {
    this->pipe.stop();
    cout << "Camera disconnected" << endl;
}

tuple<cv::Mat, cv::Mat> Camera::get_images() {
    this->frameset = this->pipe.wait_for_frames();
    this->frameset = this->align_to_color.process(this->frameset);

    rs2::video_frame color_frame = frameset.get_color_frame();
    rs2::depth_frame depth_frame = frameset.get_depth_frame();

    if (this->mat_height == 0) {
        this->mat_height = color_frame.get_height();
        this->mat_width = color_frame.get_width();
    }

    this->color_mat = cv::Mat(cv::Size(this->mat_width, this->mat_height), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
    this->depth_mat = cv::Mat(cv::Size(this->mat_width, this->mat_height), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

    // this->count++;
    return make_tuple(this->color_mat.clone(), this->depth_mat.clone());
}

void Camera::save_images(){
    this->color_filename = "../images/color_image" + to_string(this->count) + ".png";
    this->depth_filename = "../images/depth_image" + to_string(this->count) + ".png";

    imwrite(color_filename, this->color_mat);
    imwrite(depth_filename, this->depth_mat);

    cout << "Saved Image " << this->count << endl;

    string path = "../data/ImageCount.yml";
    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    if (fs.isOpened()) {
        fs << "count" << this->count;
        // fs << "R_Cam2End" << this->yamlData["R_Cam2End"];
        // fs << "T_Cam2End" << this->yamlData["T_Cam2End"];
        fs.release();
    }
}

void Camera::save_images(string color_name, string depth_name) {
    this->color_filename = color_name + to_string(this->count) + ".png";
    this->depth_filename = depth_name + to_string(this->count) + ".png";

    imwrite(color_filename, this->color_mat);
    imwrite(depth_filename, this->depth_mat);
    
    cout << "Saved Image " << this->count << endl;
    this->count++;

    string path = "../data/ImageCount.yml";
    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    if (fs.isOpened()) {
        fs << "count" << this->count;
        // fs << "R_Cam2End" << this->yamlData["R_Cam2End"];
        // fs << "T_Cam2End" << this->yamlData["T_Cam2End"];
        fs.release();
    }
}

void Camera::get_corners(bool save) {
    cvtColor(this->color_mat, this->gray_mat, cv::COLOR_BGR2GRAY);
    this->pattern_found = findChessboardCorners(this->gray_mat, this->chessboard_size, this->corners_2f);
    if(this->pattern_found){
        cornerSubPix(this->gray_mat, this->corners_2f, cv::Size(11, 11), cv::Size(-1, -1), \
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
    }
    if(save) {
        this->count++;
        drawChessboardCorners(this->color_mat, this->chessboard_size, cv::Mat(this->corners_2f), this->pattern_found);
        save_images();
    }
    this->corners_2d.clear();
    for (const auto& corner_2f : this->corners_2f)
    {
        cv::Point2d point2d(corner_2f.x, corner_2f.y);
        this->corners_2d.push_back(point2d);
        // cout << "x:" << point2d.x << "\t" << "y:" << point2d.y << endl;
    }
    // cout << "corners saved " << endl;
}

tuple<cv::Mat, cv::Mat> Camera::get_R_board2cam(bool projection) {
    solvePnP(this->corners_3d, this->corners_2d, this->intrinsics_mat, this->distCoeffs, this->V_Board2Cam, this->T_Board2Cam);
    Rodrigues(V_Board2Cam, R_Board2Cam);
    // cout << "2d: " << endl;
    // for (const auto& corner_2d : this->corners_2d)
    // {
    //     cout << "x:" << corner_2d.x << "\t" << "y:" << corner_2d.y << endl;
    // }
    // cout << "3d: " << endl;
    // for (const auto& corner_3d : this->corners_3d)
    // {
    //     cout << "x:" << corner_3d.x << "\t" << "y:" << corner_3d.y << endl;
    // }
    // cout << "Rotation vector: " << this->V_Board2Cam << endl;
    // cout << "T_Board2Cam: " << this->T_Board2Cam << endl;
    // cout << "R_Board2Cam: " << this->R_Board2Cam << endl;
    if(projection) {
        projectPoints(this->corners_3f, this->V_Board2Cam, this->T_Board2Cam, this->intrinsics_mat, this->distCoeffs, this->corners_projection_2f);
        for (size_t i = 0; i < this->corners_projection_2f.size(); i++)
        {
            cv::Point2f pt = this->corners_projection_2f[i];
            cv::circle(this->color_mat, pt, 4, cv::Scalar(0, 0, 255), -1);
        }
        save_images();
        
    }
    return make_tuple(this->R_Board2Cam.clone(), this->T_Board2Cam.clone());
}

vector<cv::Vec3f> Camera::get_circles(bool save, int mode) {
    this->centers.clear();
    cvtColor(this->color_mat, this->gray_mat, cv::COLOR_BGR2GRAY);
    // GaussianBlur(this->gray_mat, this->gray_mat, Size(3, 3), 2, 2);
    if (mode == 0)
        HoughCircles(this->gray_mat, this->circles, cv::HOUGH_GRADIENT, 0.3, this->gray_mat.rows / 30.0, 80, 12, 1, 8);
    else if (mode == 1)
        HoughCircles(this->gray_mat, this->circles, cv::HOUGH_GRADIENT, 0.5, this->gray_mat.rows / 20.0, 80, 12, 1, 12);

    // cout << this->circles.size() << endl;
    for (size_t i = 0; i < this->circles.size(); i++)
    {
        this->circle = this->circles[i];
        this->center = cv::Point2d(circle[0], circle[1]);
        this->centers.push_back(this->center);
        this->radius = this->circle[2];

        // cv::circle(this->color_mat, center, 1, cv::Scalar(0, 100, 100), 3, cv::LINE_AA);
        // cv::circle(this->color_mat, center, radius, cv::Scalar(255, 0, 255), 3, cv::LINE_AA);

    }

    if(save) {
        save_images();
    }

    // imshow("Circles: ", this->color_mat);
    // cv::waitKey(0);
    // cv::destroyAllWindows();
    
    return this->circles;
}

vector<cv::Point2d> Camera::get_ellipse(bool save, int mode)
{
    this->ellipses.clear();
    cv::Mat resized_mat, resized_gray, edges;
    double scale = 3.0;
    cv::resize(this->color_mat, resized_mat, cv::Size(this->color_mat.cols * scale, this->color_mat.rows * scale), 0, 0, cv::INTER_CUBIC);
    cv::cvtColor(resized_mat, resized_gray, cv::COLOR_BGR2GRAY);
    GaussianBlur(resized_gray, resized_gray, cv::Size(9, 9), 2, 2);
    cv::Canny(resized_gray, edges, 70, 200, 5);
    vector<vector<cv::Point>> contours;
    vector<cv::Vec4i> hierarchy;
    cv::findContours(edges, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
    
    if (save) {
        cv::imwrite("../debug/images/edges.png", edges);
    }

    vector<cv::RotatedRect> minEllipse(contours.size());
    for (int i = 0; i < contours.size(); i++)
    {
        if (contours[i].size() > 5)
        {
            minEllipse[i] = cv::fitEllipse(contours[i]);
            cv::ellipse(resized_mat, minEllipse[i], cv::Scalar(0, 255, 0), 2);
            cv::Point2d center = minEllipse[i].center;
            this->ellipses.push_back(center);
        }
    }
    // 椭圆参数缩放为原始尺寸
    for (size_t i = 0; i < this->ellipses.size(); i++)
    {
        this->ellipses[i].x /= scale;
        this->ellipses[i].y /= scale;
    }

    return this->ellipses;
}

tuple<cv::Mat, cv::Mat, vector<cv::Point2f>> Camera::get_projection_error(vector<cv::Point3d> object_points, vector<cv::Point2d> image_points){
    this->projection_error = 0;
    this->object_points_3f.clear();
    for (const auto& point3d : object_points)
    {
        cv::Point3f point3f = cv::Point3f(point3d);
        this->object_points_3f.push_back(point3f);
    }

    solvePnP(object_points, image_points, this->intrinsics_mat, this->distCoeffs, this->V_Shelf2Cam, this->T_Shelf2Cam);
    Rodrigues(this->V_Shelf2Cam, this->R_Shelf2Cam);
    projectPoints(this->object_points_3f, this->V_Shelf2Cam, this->T_Shelf2Cam, this->intrinsics_mat, this->distCoeffs, this->projection_points);
    

    for (size_t i = 0; i < image_points.size(); i++)
    {
        double dx = image_points[i].x - projection_points[i].x;
        double dy = image_points[i].y - projection_points[i].y;
        this->projection_error += sqrt(dx * dx + dy * dy);
    }
    return make_tuple(this->R_Shelf2Cam.clone(), this->T_Shelf2Cam.clone(), this->projection_points);
}

tuple<cv::Mat, cv::Mat, vector<cv::Point2f>> Camera::get_QR_pose(vector<cv::Point3d> object_points, vector<cv::Point2d> image_points){
    this->object_points_3f.clear();
    for (const auto& point3d : object_points)
    {
        cv::Point3f point3f = cv::Point3f(point3d);
        this->object_points_3f.push_back(point3f);
    }
    
    solvePnP(object_points, image_points, this->intrinsics_mat, this->distCoeffs, this->V_QR2Cam, this->T_QR2Cam);
    Rodrigues(this->V_QR2Cam, this->R_QR2Cam);
    projectPoints(this->object_points_3f, this->V_QR2Cam, this->T_QR2Cam, this->intrinsics_mat, this->distCoeffs, this->projection_points);

    return make_tuple(this->R_QR2Cam.clone(), this->T_QR2Cam.clone(), this->projection_points);
}

tuple<cv::Mat, cv::Mat, vector<cv::Point2f>, string> Camera::get_shelf_pose(vector<vector<cv::Point3d>> object_points, vector<cv::Point2d> center_points, bool & pose_flag) {
    random_device rd;
    mt19937 gen(rd());
    tuple<cv::Mat, cv::Mat, vector<cv::Point2f>> shelf_pose;
    // 计时
    auto start = chrono::high_resolution_clock::now();
    double min_x, max_x, min_y, max_y;
    try {
        for (size_t iteration = 0; iteration < 10000; iteration++)
        {
            // 计时
            auto end = chrono::high_resolution_clock::now();
            chrono::duration<double> elapsed = end - start;
            start = end;
            if (elapsed.count() > 5)
            {
                std::cout  << "超时" << std::endl;
                return tuple_cat(shelf_pose, make_tuple("none"));
            }
            
            do
            {
                this->center_points_5.clear();
                min_x = 1000;
                min_y = 1000;
                max_x = -1;
                max_y = -1;
                shuffle(center_points.begin(), center_points.end(), gen);
                for (size_t j = 0; j < 5; j++)
                {
                    this->center_points_5.push_back(center_points[j]);
                }
                for (auto point : center_points_5)
                {
                    min_x = min(min_x, point.x);
                    min_y = min(min_y, point.y);
                    max_x = max(max_x, point.x);
                    max_y = max(max_y, point.y);
                }
            } while (max_x - min_x > 180 || max_y - min_y > 100);
            
            // criterion up 左3 右2 反方向-------------------------------------------------------------------------------------------------------------------
            // sort by x
            sort(this->center_points_5.begin(), this->center_points_5.end(), [](const cv::Point2d& p1, const cv::Point2d& p2){
                return p1.x < p2.x;
            });
            this->image_points.clear();
            this->center_points_copy = this->center_points_5;
            for (size_t i = 0; i < 3; i++)
            {
                auto minPoint = min_element(this->center_points_copy.begin(), this->center_points_copy.begin() + 3 - i, \
                [](const cv::Point2d& p1, const cv::Point2d& p2) {
                    return p1.y < p2.y;
                });
                this->image_points.push_back(*minPoint);
                this->center_points_copy.erase(minPoint);
            }
            for (size_t i = 0; i < 2; i++)
            {
                auto minPoint = min_element(this->center_points_copy.begin(), this->center_points_copy.end(), \
                [](const cv::Point2d& p1, const cv::Point2d& p2) {
                    return p1.y < p2.y;
                });
                this->image_points.push_back(*minPoint);
                this->center_points_copy.erase(minPoint);
            }
            auto distance = calculateMinDistance(this->image_points);
            shelf_pose = get_projection_error(object_points[0], this->image_points);
            if (this->projection_error < 2 and distance > 2) {
                cout << "projection error: " << this->projection_error << endl;
                pose_flag = true; 
                return tuple_cat(shelf_pose, make_tuple("upN"));
            }

            // criterion up 2 左2 右3 正方向-------------------------------------------------------------------------------------------------------------------
            sort(this->center_points_5.begin(), this->center_points_5.end(), [](const cv::Point2d& p1, const cv::Point2d& p2){
                return p1.x > p2.x;
            });
            this->image_points.clear();
            this->center_points_copy = this->center_points_5;
            for (size_t i = 0; i < 3; i++)
            {
                auto minPoint = min_element(this->center_points_copy.begin(), this->center_points_copy.begin() + 3 - i, \
                [](const cv::Point2d& p1, const cv::Point2d& p2) {
                    return p1.y > p2.y;
                });
                this->image_points.push_back(*minPoint);
                this->center_points_copy.erase(minPoint);
            }
            for (size_t i = 0; i < 2; i++)
            {
                auto minPoint = min_element(this->center_points_copy.begin(), this->center_points_copy.end(), \
                [](const cv::Point2d& p1, const cv::Point2d& p2) {
                    return p1.y > p2.y;
                });
                this->image_points.push_back(*minPoint);
                this->center_points_copy.erase(minPoint);
            }
            distance = calculateMinDistance(this->image_points);
            shelf_pose = get_projection_error(object_points[0], this->image_points);
            if (this->projection_error < 2 and distance > 2) {
                cout << "projection error: " << this->projection_error << endl;
                pose_flag = true; 
                return tuple_cat(shelf_pose, make_tuple("upP"));
            }

            // criterion up 上3 下2 头朝左-------------------------------------------------------------------------------------------------------------------
            sort(this->center_points_5.begin(), this->center_points_5.end(), [](const cv::Point2d& p1, const cv::Point2d& p2){
                return p1.y < p2.y;
            });
            this->image_points.clear();
            this->center_points_copy = this->center_points_5;
            for (size_t i = 0; i < 3; i++)
            {
                auto minPoint = min_element(this->center_points_copy.begin(), this->center_points_copy.begin() + 3 - i, \
                [](const cv::Point2d& p1, const cv::Point2d& p2) {
                    return p1.x > p2.x;
                });
                this->image_points.push_back(*minPoint);
                this->center_points_copy.erase(minPoint);
            }
            for (size_t i = 0; i < 2; i++)
            {
                auto minPoint = min_element(this->center_points_copy.begin(), this->center_points_copy.end(), \
                [](const cv::Point2d& p1, const cv::Point2d& p2) {
                    return p1.x > p2.x;
                });
                this->image_points.push_back(*minPoint);
                this->center_points_copy.erase(minPoint);
            }
            distance = calculateMinDistance(this->image_points);
            shelf_pose = get_projection_error(object_points[0], this->image_points);
            if (this->projection_error < 2 and distance > 2) {
                cout << "projection error: " << this->projection_error << endl;
                pose_flag = true; 
                return tuple_cat(shelf_pose, make_tuple("upL"));
            }

            // criterion up 上2 下3 头朝右-------------------------------------------------------------------------------------------------------------------
            sort(this->center_points_5.begin(), this->center_points_5.end(), [](const cv::Point2d& p1, const cv::Point2d& p2){
                return p1.y > p2.y;
            });
            this->image_points.clear();
            this->center_points_copy = this->center_points_5;
            for (size_t i = 0; i < 3; i++)
            {
                auto minPoint = min_element(this->center_points_copy.begin(), this->center_points_copy.begin() + 3 - i, \
                [](const cv::Point2d& p1, const cv::Point2d& p2) {
                    return p1.x < p2.x;
                });
                this->image_points.push_back(*minPoint);
                this->center_points_copy.erase(minPoint);
            }
            for (size_t i = 0; i < 2; i++)
            {
                auto minPoint = min_element(this->center_points_copy.begin(), this->center_points_copy.end(), \
                [](const cv::Point2d& p1, const cv::Point2d& p2) {
                    return p1.x < p2.x;
                });
                this->image_points.push_back(*minPoint);
                this->center_points_copy.erase(minPoint);
            }
            distance = calculateMinDistance(this->image_points);
            shelf_pose = get_projection_error(object_points[0], this->image_points);
            if (this->projection_error < 2 and distance > 2) {
                cout << "projection error: " << this->projection_error << endl;
                pose_flag = true; 
                return tuple_cat(shelf_pose, make_tuple("upR"));
            }

            // criterion right L 左上1 中3 中下1 -------------------------------------------------------------------------------------------------------------------
            this->image_points.clear();
            sort(this->center_points_5.begin(), this->center_points_5.end(), [](const cv::Point2d& p1, const cv::Point2d& p2){
                return p1.y > p2.y;
            });
            this->center_points_copy = this->center_points_5;
            this->image_points.push_back(this->center_points_copy[0]);
            this->image_points.push_back(this->center_points_copy[4]);
            this->center_points_copy.erase(center_points_copy.begin());
            this->center_points_copy.erase(center_points_copy.begin() + 3);
            for (size_t i = 0; i < 2; i++)
            {
                auto minPoint = min_element(this->center_points_copy.begin(), this->center_points_copy.end(), \
                [](const cv::Point2d& p1, const cv::Point2d& p2) {
                    return p1.x < p2.x;
                });
                this->image_points.push_back(*minPoint);
                this->center_points_copy.erase(minPoint);
            }
            this->image_points.push_back(this->center_points_copy[0]);
            distance = calculateMinDistance(this->image_points);
            shelf_pose = get_projection_error(object_points[1], this->image_points);
            if (this->projection_error < 2 and distance > 2) {
                cout << "projection error: " << this->projection_error << endl;
                return tuple_cat(shelf_pose, make_tuple("rightL"));
            }
            // criterion right P 左2 中2 中下1 -------------------------------------------------------------------------------------------------------------------
            this->image_points.clear();
            sort(this->center_points_5.begin(), this->center_points_5.end(), [](const cv::Point2d& p1, const cv::Point2d& p2){
                return p1.x < p2.x;
            });
            this->center_points_copy = this->center_points_5;
            this->image_points.push_back(this->center_points_copy[0]);
            this->image_points.push_back(this->center_points_copy[4]);
            this->center_points_copy.erase(center_points_copy.begin());
            this->center_points_copy.erase(center_points_copy.begin() + 3);
            for (size_t i = 0; i < 2; i++)
            {
                auto minPoint = min_element(this->center_points_copy.begin(), this->center_points_copy.end(), \
                [](const cv::Point2d& p1, const cv::Point2d& p2) {
                    return p1.y < p2.y;
                });
                this->image_points.push_back(*minPoint);
                this->center_points_copy.erase(minPoint);
            }
            this->image_points.push_back(this->center_points_copy[0]);
            distance = calculateMinDistance(this->image_points);
            shelf_pose = get_projection_error(object_points[1], this->image_points);
            if (this->projection_error < 2 and distance > 2) {
                cout << "projection error: " << this->projection_error << endl;
                return tuple_cat(shelf_pose, make_tuple("rightP"));
            }
            // criterion right N 左下1 中3 右中1 -------------------------------------------------------------------------------------------------------------------
            this->image_points.clear();
            sort(this->center_points_5.begin(), this->center_points_5.end(), [](const cv::Point2d& p1, const cv::Point2d& p2){
                return p1.x > p2.x;
            });
            this->center_points_copy = this->center_points_5;
            this->image_points.push_back(this->center_points_copy[0]);
            this->image_points.push_back(this->center_points_copy[4]);
            this->center_points_copy.erase(center_points_copy.begin());
            this->center_points_copy.erase(center_points_copy.begin() + 3);
            for (size_t i = 0; i < 2; i++)
            {
                auto minPoint = min_element(this->center_points_copy.begin(), this->center_points_copy.end(), \
                [](const cv::Point2d& p1, const cv::Point2d& p2) {
                    return p1.y > p2.y;
                });
                this->image_points.push_back(*minPoint);
                this->center_points_copy.erase(minPoint);
            }
            this->image_points.push_back(this->center_points_copy[0]);
            distance = calculateMinDistance(this->image_points);
            shelf_pose = get_projection_error(object_points[1], this->image_points);
            if (this->projection_error < 2 and distance > 2) {
                cout << "projection error: " << this->projection_error << endl;
                return tuple_cat(shelf_pose, make_tuple("rightN"));
            }

            // criterion left R 上2 中1 下2-------------------------------------------------------------------------------------------------------------------
           this->image_points.clear();
            sort(this->center_points_5.begin(), this->center_points_5.end(), [](const cv::Point2d& p1, const cv::Point2d& p2){
                return p1.y < p2.y;
            });
            this->center_points_copy = this->center_points_5;
            for (size_t i = 0; i < 2; i++)
            {
                auto minPoint = min_element(this->center_points_copy.begin(), this->center_points_copy.begin() + 2 - i, \
                [](const cv::Point2d& p1, const cv::Point2d& p2) {
                    return p1.x > p2.x;
                });
                this->image_points.push_back(*minPoint);
                this->center_points_copy.erase(minPoint);
            }
            this->image_points.push_back(this->center_points_copy[0]);
            this->center_points_copy.erase(this->center_points_copy.begin());
            for (size_t i = 0; i < 2; i++)
            {
                auto minPoint = min_element(this->center_points_copy.begin(), this->center_points_copy.end(), \
                [](const cv::Point2d& p1, const cv::Point2d& p2) {
                    return p1.x < p2.x;
                });
                this->image_points.push_back(*minPoint);
                this->center_points_copy.erase(minPoint);
            }
            distance = calculateMinDistance(this->image_points);
            shelf_pose = get_projection_error(object_points[2], this->image_points);
            if (this->projection_error < 2 and distance > 2) {
                cout << "projection error: " << this->projection_error << endl;
                return tuple_cat(shelf_pose, make_tuple("leftR"));
            }
            // criterion left P 上1 左2 右2-------------------------------------------------------------------------------------------------------------------
            this->image_points.clear();
            sort(this->center_points_5.begin(), this->center_points_5.end(), [](const cv::Point2d& p1, const cv::Point2d& p2){
                return p1.x < p2.x;
            });
            this->center_points_copy = this->center_points_5;
            for (size_t i = 0; i < 2; i++)
            {
                auto minPoint = min_element(this->center_points_copy.begin(), this->center_points_copy.begin() + 2 - i, \
                [](const cv::Point2d& p1, const cv::Point2d& p2) {
                    return p1.y < p2.y;
                });
                this->image_points.push_back(*minPoint);
                this->center_points_copy.erase(minPoint);
            }
            this->image_points.push_back(this->center_points_copy[0]);
            this->center_points_copy.erase(this->center_points_copy.begin());
            for (size_t i = 0; i < 2; i++)
            {
                auto minPoint = min_element(this->center_points_copy.begin(), this->center_points_copy.end(), \
                [](const cv::Point2d& p1, const cv::Point2d& p2) {
                    return p1.y < p2.y;
                });
                this->image_points.push_back(*minPoint);
                this->center_points_copy.erase(minPoint);
            }
            distance = calculateMinDistance(this->image_points);
            shelf_pose = get_projection_error(object_points[2], this->image_points);
            if (this->projection_error < 2 and distance > 2) {
                cout << "projection error: " << this->projection_error << endl;
                return tuple_cat(shelf_pose, make_tuple("leftP"));
            }

            // for (Point2d image_point : image_points)
            // {
            //     cout << image_point.x << "\t" << image_point.y << endl;
            // }
        }
    }
    catch(const std::exception& e)
    {
        // throw runtime_error("can't find projection !!!");
        std::cerr << "can't find projection !!! " << e.what() << '\n';
        return tuple_cat(shelf_pose, make_tuple("none"));
    }
    pose_flag = false;
    return tuple_cat(shelf_pose, make_tuple("none"));
}

double Camera::getDistance(int x, int y) {
    return this->frameset.get_depth_frame().get_distance(x, y);
}

int Camera::get_img_count()
{
    return this->count;
}

double calculateMinDistance(const std::vector<cv::Point2d>& points) {
    if (points.size() < 2) {
        throw std::invalid_argument("At least two points are required");
    }

    double minDistance = std::numeric_limits<double>::max();

    for (size_t i = 0; i < points.size(); ++i) {
        for (size_t j = i + 1; j < points.size(); ++j) {
            double dx = points[i].x - points[j].x;
            double dy = points[i].y - points[j].y;
            double distance = std::sqrt(dx * dx + dy * dy);

            if (distance < minDistance) {
                minDistance = distance;
            }
        }
    }

    return minDistance;
}

