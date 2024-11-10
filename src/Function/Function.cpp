#include "Function.h"
#include <QDebug>
#include <QThread>
#include <QTimer>
#include <opencv2/calib3d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <cmath>
#include <opencv2/aruco/charuco.hpp>


Function::Function(QObject *parent) : QObject(parent)
{

}

Function::~Function()
{
}

void Function::recv_mat(cv::Mat mat)
{
    current_mat = mat;
}

void Function::init_UR5_position(UR5 * ur5)
{
    // vector<double> init_pose = {-0.0619756, -0.443327, 0.407511, -0.0199297, -3.12949, 0.00665348}; // 水平向下看
    // vector<double> init_pose = { 0.270617, -0.429232, 0.371516, -0.777165, 2.70524, 0.0447376}; // 斜着看
    // test 在 0-3 反复变化
    if (test == 0)
    {
        init_pose = init_poses[0];
        test = 1;
    }
    else if (test == 1)
    {
        init_pose = init_poses[1];
        test = 2;
    }
    else if (test == 2)
    {
        init_pose = init_poses[2];
        test = 3;
    }
    else if (test == 3)
    {
        init_pose = init_poses[1];
        test = 0;
    }

    ur5->moveJP(init_pose);
    emit init_pose_change((test + 3) % 4);
    emit init_grasp_finished();
}

void Function::change_init_pose(int index)
{
    if (index == 0)
    {
        init_pose = init_poses[0];
    }
    else if (index == 1)
    {
        init_pose = init_poses[1];
    }
    else if (index == 2)
    {
        init_pose = init_poses[2];
    }
    else if (index == 3)
    {
        init_pose = init_poses[1];
    }
}

void Function::shelf_grasp(Camera * camera, UR5 * ur5, cv::Mat matrix) {
    qDebug() << "子线程" << QThread::currentThreadId();
    // object points
    vector<vector<Point3d>> object_points = {
    // {Point3d(0.00, 0.00, 0.00), Point3d(0.00, 0.01, 0.00), Point3d(0.00, 0.02, 0.00), Point3d(0.05, 0.00, 0.00), Point3d(0.05, 0.02, 0.00)},
    // {Point3d(0.00, 0.01, 0.00), Point3d(0.01, 0.00, 0.00), Point3d(0.01, 0.01, 0.00), Point3d(0.01, 0.02, 0.00), Point3d(0.02, 0.00, 0.00)},
    // {Point3d(0.00, 0.01, 0.00), Point3d(0.00, 0.02, 0.00), Point3d(0.01, 0.00, 0.00), Point3d(0.02, 0.01, 0.00), Point3d(0.02, 0.02, 0.00)},
    
    // 试管架上方的点：5个点中心为原点
    {Point3d(-0.025, -0.01, 0.00), Point3d(-0.025, 0.00, 0.00), Point3d(-0.025, 0.01, 0.00), Point3d(0.025, -0.01, 0.00), Point3d(0.025, 0.01, 0.00)},
    // 试管架上方的点：5个点第一个为原点
    // {Point3d(0.00, 0.00, 0.00), Point3d(0.00, 0.01, 0.00), Point3d(0.00, 0.02, 0.00), Point3d(0.05, 0.00, 0.00), Point3d(0.05, 0.02, 0.00)},
    // 试管架右侧的点: 5个点中心为原点
    {Point3d(-0.01, 0.00, 0.00), Point3d(0.01, -0.01, 0.00), Point3d(0.00, -0.01, 0.00), Point3d(0.00, 0.00, 0.00), Point3d(0.00, 0.01, 0.00)},
    // 试管架右侧的点: 5个点第一个为原点
    // {Point3d(0.00, 0.01, 0.00), Point3d(0.02, 0.00, 0.00), Point3d(0.01, 0.00, 0.00), Point3d(0.01, 0.01, 0.00), Point3d(0.01, 0.02, 0.00)},
    // 试管架左侧的点: 5个点中心为原点
    {Point3d(-0.01, 0.00, 0.00), Point3d(-0.01, 0.01, 0.00), Point3d(0.00, -0.01, 0.00), Point3d(0.01, 0.00, 0.00), Point3d(0.01, 0.01, 0.00)},
    // 试管架左侧的点: 5个点第一个为原点
    // {Point3d(0.00, 0.01, 0.00), Point3d(0.00, 0.02, 0.00), Point3d(0.01, 0.00, 0.00), Point3d(0.02, 0.01, 0.00), Point3d(0.02, 0.02, 0.00)},
    };
    
    // image points
    vector<Point2d> center_points_all, center_points_5;
    // vector<Vec3f> circles;
    vector<cv::Point2d> circles;
    Point2d center;
    // Vec3f circle;
    Point2d circle;
    vector<Point2f> projection_points;
    int radius;
    // image processing
    Mat color_mat, hsv_mat, mask_low, mask_high, mask, gray_mat, shelf_mask;
    Mat labels, stats, centroids;
    Mat intrinsics_mat, distCoeffs;
    tuple<Mat, Mat> images;
    // calculate pose
    Mat V_Shelf2Cam, R_Shelf2Cam, T_Shelf2Cam, RT_Shelf2Cam, RT_Shelf2Base, RT_End2Base;
    Mat R_Cam2End, T_Cam2End, RT_Cam2End;

    // vector<double> init_pose = {-0.0619756, -0.443327, 0.407511, -0.0199297, -3.12949, 0.00665348}; // 水平向下看
    // vector<double> init_pose = { 0.270617, -0.429232, 0.371516, -0.777165, 2.70524, 0.0447376}; // 斜着看
    
    // 上方视角的目标点
    // 目标点1 5个点第一个为原点
    // Mat P_Shelf_Up_1 = (Mat_<double>(4, 1) << 0.025, -0.08, -0.200, 1.0);
    // 目标点1 5个点中心为原点
    Mat P_Shelf_Up_1 = (Mat_<double>(4, 1) << 0.00, -0.09, -0.200, 1.0);
    // 目标点2 5个点第一个为原点
    // Mat P_Shelf_Up_2 = (Mat_<double>(4, 1) << 0.025, 0.01, -0.040, 1.0);
    // 目标点3 5个点第一个为原点
    // Mat P_Shelf_Up_3 = (Mat_<double>(4, 1) << 0.025, 0.01, -0.030, 1.0);
    // 目标点2 5个点中心为原点
    Mat P_Shelf_Up_2 = (Mat_<double>(4, 1) << 0.00, 0.00, -0.040, 1.0);
    // 目标点3 5个点中心为原点
    Mat P_Shelf_Up_3 = (Mat_<double>(4, 1) << 0.00, 0.00, -0.030, 1.0);

    // 看试管架右侧时的目标点
    Mat P_Shelf_Right_1 = (Mat_<double>(4, 1) << 0.00, -0.220, 0.160, 1.0); // 第二次转到上方观察

    // 看试管架左侧时的目标点
    Mat P_Shelf_Left_1 = (Mat_<double>(4, 1) << 0.00, -0.220, 0.160, 1.0); // 第二次转到上方观察

    // 试管架坐标系下，目标点的旋转矩阵均为单位矩阵
    Mat P_Shelf_Rotation = (Mat_<double>(3, 3) << 
        1, 0, 0, 
        0, 1, 0, 
        0, 0, 1
    );
    // 看试管架右侧时，目标点的旋转矩阵, 绕x轴顺时针旋转90度
    Mat P_Shelf_Right_Rotation = (Mat_<double>(3, 3) << 
        0, 0, 1, 
        -1, 0, 0, 
        0, -1, 0
    );
    // 看试管架左侧时，目标点的旋转矩阵, 绕x轴顺时针旋转90度
    // 同上

    Mat target_P_1, target_P_2, target_P_3;
    // vector<double> orign_rotation = {init_pose[3], init_pose[4], init_pose[5]};
    // vector<double> target_rotation = {0.023, -3.130, 0.010};
    // vector<double> target_rotation = {-0.0204577, -3.13029, 0.0247043};
    
    vector<double> target_pose_1, target_pose_2, target_pose_3;

    string color_filename;

    double projection_error;
    tuple<Mat, Mat, vector<Point2f>, string> shelf_pose;

    double x_max = 0.13275;
    double x_min = -0.20079;
    double y_max = -0.38654;
    double y_min = -0.68844;
    double z_max = 0.45000;
    double z_min = 0.09247;

    bool save = true;
    bool projection = true;
    string type;
    try {
    // 读取相机到末端的位姿
    // cv::FileStorage fs_read("../data/Cam2End.yml", FileStorage::READ);
    // if (fs_read.isOpened()) {
    //     fs_read["R_Cam2End"] >> R_Cam2End;
    //     fs_read["T_Cam2End"] >> T_Cam2End;
    //     fs_read.release();
    // }
    // RT_Cam2End = ur5->R_and_T2RT(R_Cam2End, T_Cam2End);
    // std::cout << "RT_Cam2End: " << RT_Cam2End << std::endl;

    // 移动到初始位置
    ur5->moveJP(init_pose);

    int times= 1;
    for (int loop = 0; loop < 2; loop++)
    {
        while (1) 
        {
            std::cout << "第" << times << "次识别------------------------------" << std::endl;
            // 获取图片
            images = camera->get_images();
            color_mat = get<0>(images);

            // 获取圆
            // circles = camera->get_circles(save, 0);
            circles = camera->get_ellipse(save, 0);

            // 滤波
            cv::medianBlur(color_mat, color_mat, 3);
            cv::imwrite("../debug/images/"+to_string(loop+1)+"img1-original.png", color_mat);
            std::cout << "存入原始图片" << std::endl;

            // 颜色识别提取
            cv::cvtColor(color_mat, hsv_mat, cv::COLOR_BGR2HSV);
            // cv::inRange(hsv_mat, cv::Scalar(0, 0, 0), cv::Scalar(10, 255, 255), mask_low);
            // cv::inRange(hsv_mat, cv::Scalar(160, 0, 0), cv::Scalar(179, 255, 255), mask_high);
            cv::inRange(hsv_mat, cv::Scalar(0, 43, 46), cv::Scalar(10, 255, 255), mask_low);
            cv::inRange(hsv_mat, cv::Scalar(156, 43, 46), cv::Scalar(180, 255, 255), mask_high);
            cv::addWeighted(mask_low, 1, mask_high, 1, 0, mask);
            cv::imwrite("../debug/images/"+to_string(loop+1)+"img2-mask.png", mask);
            std::cout << "存入mask图片" << std::endl;

            // 形态学处理
            cv::Mat element1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
            cv::Mat element2 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2));
            cv::dilate(mask, mask, element1);
            cv::imwrite("../debug/images/"+to_string(loop+1)+"img3-morphomask.png", mask);
            std::cout << "存入形态学处理后的mask图片" << std::endl;

            // 画出筛选之前的圆
            cv::Mat color_copy = color_mat.clone();
            for (size_t i = 0; i < circles.size(); i++)
            {
                // if (mask.at<uchar>(cvRound(circles[i][1]), cvRound(circles[i][0])) == 255)
                // {
                    circle = circles[i];
                    // center = Point2d(circle[0], circle[1]);
                    center = circle;
                    // radius = circle[2];
                    radius = 3;
                    cv::circle(color_copy, center, radius, cv::Scalar(255, 0, 255), 1, cv::LINE_AA);
                // }
            }
            std::cout << "筛选前圆的数量" << circles.size() << std::endl;
            cv::imwrite("../debug/images/"+to_string(loop+1)+"img4-circle-before.png", color_copy);
            std::cout << "存入筛选前查找的圆的图片" << std::endl;

            center_points_all.clear();
            // 筛选圆，选出红色的圆
            cv::Mat color_copy2 = color_mat.clone();
            for (size_t i = 0; i < circles.size(); i++)
            {
                circle = circles[i];
                // 防止数组越界
                if (circle.x < 0 || circle.y < 0 || circle.x > color_mat.cols || circle.y > color_mat.rows)
                {
                    continue;
                }
                if (
                    mask.at<uchar>(cvRound(circle.y), cvRound(circle.x)) == 255
                //     // circles[i][1] > rect.y && circles[i][1] < rect.y + rect.height &&
                //     // circles[i][0] > rect.x && circles[i][0] < rect.x + rect.width
                )
                {
                    if (loop == 1) { // 第二次识别
                        if (circle.x < 156 || circle.x > 550 || circle.y < 104 || circle.y > 355)
                        {
                            continue;
                        }
                    }
                    // center = cv::Point2d(circle[0], circle[1]);
                    center = circle;
                    center_points_all.push_back(center);
                    radius = 3;
                    cv::circle(color_copy2, center, radius, cv::Scalar(255, 0, 255), 1, cv::LINE_AA);
                }
            }

            if (center_points_all.size() < 5)
            {
                times++;
                // sleep(1);
                continue; // 重新识别
            }

            std::cout << "筛选后圆的数量" << center_points_all.size() << std::endl;
            std::cout << "存入筛选后查找的圆的图片" << std::endl;
            imwrite("../debug/images/"+to_string(loop+1)+"img5-circle-after.png", color_copy2);
            std::cout << "本次识别圆的数量" << center_points_all.size() << std::endl;

            // 启动定时器计时，长时间找不到就轻微移动机械臂重新识别
            // 获取试管架位姿
            std::cout << "开始计算位姿..." << std::endl;
            bool get_pose = false;
            shelf_pose = camera->get_shelf_pose(object_points, center_points_all, get_pose);
            string type = get<3>(shelf_pose);
            if (type == "none")
            {
                std::cout << "识别失败，重新识别" << std::endl;
                center_points_all.clear();
                // auto current_pose_tuple = ur5->getCurrentPositions();
                // vector<double> current_pose = get<1>(current_pose_tuple);
                // if (times % 2 == 0)
                // {
                //     current_pose[2] -= 0.01; // z轴下降10mm后重新识别
                // }
                // else
                // {
                //     current_pose[2] += 0.01; // z轴上升10mm后重新识别
                // }
                // ur5->moveJP(current_pose);
                times++;
                // sleep(1);
                continue;
            }
            else {
                break;
            }
        }
        /*
        旋转：
            末端坐标系中，末端点的坐标为(0, 0, 0)
            基坐标系下的一个点
            架子相对于基座的旋转矩阵 = 架子相对于相机的旋转矩阵 * 相机相对于末端的旋转矩阵 * 末端相对于基座的旋转矩阵
        */
        
        // 获取试管架和相机之间的变换矩阵
        type = get<3>(shelf_pose);
        cout << type << endl;
        R_Shelf2Cam = get<0>(shelf_pose); // 试管架物理坐标系到相机坐标系的旋转矩阵
        T_Shelf2Cam = get<1>(shelf_pose);
        projection_points = get<2>(shelf_pose);

        // 绘制重投影点
        for (size_t i = 0; i < projection_points.size(); i++)
        {
            Point2f pt = projection_points[i];
            cv::circle(color_mat, pt, 4, Scalar(0, 0, 255), -1);
            // 标上序号
            cv::putText(color_mat, to_string(i+1), pt, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
            // cout << pt.x << "\t" << pt.y << endl;
        }

        // 计算试管架朝向角度，根据重投影的5点计算, 第三个点减去第一个点的向量 和 图像中-y方向的向量之间的夹角


        cout << "存入重投影的图片" << endl;
        imwrite("../debug/images/"+to_string(loop+1)+"img6-projection.png", color_mat);
        // imshow("projection: ", color_mat);
        color_filename = "../images/projection.png";
        imwrite(color_filename, color_mat); 
        times++;

        // 计算目标点的位姿
        // 获取试管架到相机的变换矩阵
        RT_Shelf2Cam = ur5->R_and_T2RT(R_Shelf2Cam, T_Shelf2Cam);

        // 获取相机到末端的变换矩阵
        cv::FileStorage fs_read("../data/Cam2End.yml", FileStorage::READ);
        if (fs_read.isOpened()) {
            fs_read["R_Cam2End"] >> R_Cam2End;
            fs_read["T_Cam2End"] >> T_Cam2End;
            fs_read.release();
        }
        RT_Cam2End = ur5->R_and_T2RT(R_Cam2End, T_Cam2End);

        // 获取末端到基座的变换矩阵
        tuple<cv::Mat, cv::Mat> array2 = ur5->getCurrentRT();
        RT_End2Base = ur5->R_and_T2RT(get<0>(array2), get<1>(array2));

        std::cout << "----------------------------------开始移动-----------------------------------" <<endl;

        // 根据具体情况选择旋转方式
        cv::Mat RT = cv::Mat::eye(4, 4, CV_64F);
        if (type == "upN") // 目标点绕试管架坐标系z轴旋转180度
        {
            // RT = getRotationMatrixAroundAxis(0.025, 0.01, 1, 180);
            RT = getRotationMatrixAroundAxis(0, 0, 1, 180);
        }
        else if (type == "upL") // 目标点绕绕试管架坐标系z轴顺时针旋转90度
        {
            if (projection_points[2].y >  projection_points[0].y)
            {
                RT = getRotationMatrixAroundAxis(0, 0, 1, 180);
            }
            // RT = getRotationMatrixAroundAxis(0.025, 0.01, 1, 90);
            // RT = getRotationMatrixAroundAxis(0, 0, 1, 90);
        }
        else if (type == "upR") // 目标点绕绕试管架坐标系z轴逆时针旋转90度
        {
            if (projection_points[2].y >  projection_points[0].y)
            {
                RT = getRotationMatrixAroundAxis(0, 0, 1, 180);
            }
            // RT = getRotationMatrixAroundAxis(0.025, 0.01, 1, -90);
            // RT = getRotationMatrixAroundAxis(0, 0, 1, -90);
        }
        else if (type == "rightN") // 看到的是试管架右侧的点时
        {
            RT = getRotationMatrixAroundAxis(0, 0, 1, -90);
            // std::cout << RT << std::endl;
        }

        if (loop == 0) {
            // 第一个点
            if (type == "upN" || type == "upP" || type == "upL" || type == "upR")
            {
                cv::Mat ShelfRT_1 = ur5->R_and_T2RT(P_Shelf_Rotation, P_Shelf_Up_1.rowRange(0, 3)); 
                target_P_1 = RT * ShelfRT_1;
                target_P_1 = RT_End2Base * RT_Cam2End * RT_Shelf2Cam * target_P_1;
            }
            else if (type == "rightL" || type == "rightR" || type == "rightP" || type == "rightN")
            {
                cv::Mat ShelfRT_1 = ur5->R_and_T2RT(P_Shelf_Right_Rotation, P_Shelf_Right_1.rowRange(0, 3)); 
                cv::Mat E = cv::Mat::eye(4, 4, CV_64F);
                target_P_1 = E * ShelfRT_1;
                target_P_1 = RT_End2Base * RT_Cam2End * RT * RT_Shelf2Cam * target_P_1;
            }
            else if (type == "leftR" || type == "leftL" || type == "leftP" || type == "leftN")
            {
                cv::Mat ShelfRT_1 = ur5->R_and_T2RT(P_Shelf_Right_Rotation, P_Shelf_Right_1.rowRange(0, 3)); 
                cv::Mat E = cv::Mat::eye(4, 4, CV_64F);
                target_P_1 = E * ShelfRT_1;
                target_P_1 = RT_End2Base * RT_Cam2End * RT_Shelf2Cam * target_P_1;
            }

            cv::Mat target_P_1_T(3, 1, CV_64F); 
            target_P_1_T.at<double>(0, 0) = target_P_1.at<double>(0, 3);  // 提取平移向量
            target_P_1_T.at<double>(1, 0) = target_P_1.at<double>(1, 3);  
            target_P_1_T.at<double>(2, 0) = target_P_1.at<double>(2, 3);
            target_pose_1.push_back(target_P_1_T.at<double>(0));
            target_pose_1.push_back(target_P_1_T.at<double>(1));
            target_pose_1.push_back(target_P_1_T.at<double>(2));

            cv::Mat target_P_1_R = target_P_1(cv::Rect(0, 0, 3, 3)); // 提取旋转矩阵
            cv::Mat target_rotation_1(3, 1, CV_64F); // 旋转矩阵转旋转向量
            cv::Rodrigues(target_P_1_R, target_rotation_1);
            target_pose_1.push_back(target_rotation_1.at<double>(0));
            target_pose_1.push_back(target_rotation_1.at<double>(1));
            target_pose_1.push_back(target_rotation_1.at<double>(2));

            for (auto element : target_pose_1)
            {
                cout << element << ", ";
            }std::cout << endl;
            ur5->moveJP(target_pose_1);
        }
        else {
            // 第二个点
            cv::Mat ShelfRT_2 = ur5->R_and_T2RT(P_Shelf_Rotation, P_Shelf_Up_2.rowRange(0, 3));
            // 第三个点
            cv::Mat ShelfRT_3 = ur5->R_and_T2RT(P_Shelf_Rotation, P_Shelf_Up_3.rowRange(0, 3));

            target_P_2 = RT * ShelfRT_2;
            target_P_3 = RT * ShelfRT_3;

            target_P_2 = RT_End2Base * RT_Cam2End * RT_Shelf2Cam * target_P_2;
            target_P_3 = RT_End2Base * RT_Cam2End * RT_Shelf2Cam * target_P_3;

            cv:Mat target_P_2_T(3, 1, CV_64F);
            target_P_2_T.at<double>(0, 0) = target_P_2.at<double>(0, 3);  // 提取平移向量
            target_P_2_T.at<double>(1, 0) = target_P_2.at<double>(1, 3);  
            target_P_2_T.at<double>(2, 0) = target_P_2.at<double>(2, 3);  
            target_pose_2.push_back(target_P_2_T.at<double>(0));
            target_pose_2.push_back(target_P_2_T.at<double>(1));
            target_pose_2.push_back(target_P_2_T.at<double>(2));

            cv::Mat target_P_2_R = target_P_2(cv::Rect(0, 0, 3, 3)); // 提取旋转矩阵
            cv::Mat target_rotation_2(3, 1, CV_64F); // 旋转矩阵转旋转向量
            cv::Rodrigues(target_P_2_R, target_rotation_2);
            target_pose_2.push_back(target_rotation_2.at<double>(0));
            target_pose_2.push_back(target_rotation_2.at<double>(1));
            target_pose_2.push_back(target_rotation_2.at<double>(2));

            cv::Mat target_P_3_T(3, 1, CV_64F);
            target_P_3_T.at<double>(0, 0) = target_P_3.at<double>(0, 3);  // 提取平移向量
            target_P_3_T.at<double>(1, 0) = target_P_3.at<double>(1, 3);  
            target_P_3_T.at<double>(2, 0) = target_P_3.at<double>(2, 3);  
            target_pose_3.push_back(target_P_3_T.at<double>(0));
            target_pose_3.push_back(target_P_3_T.at<double>(1));
            target_pose_3.push_back(target_P_3_T.at<double>(2));

            cv::Mat target_P_3_R = target_P_3(cv::Rect(0, 0, 3, 3)); // 提取旋转矩阵
            cv::Mat target_rotation_3(3, 1, CV_64F); // 旋转矩阵转旋转向量
            cv::Rodrigues(target_P_3_R, target_rotation_3);
            target_pose_3.push_back(target_rotation_3.at<double>(0));
            target_pose_3.push_back(target_rotation_3.at<double>(1));
            target_pose_3.push_back(target_rotation_3.at<double>(2));

            for (auto element : target_pose_2)
            {
                cout << element << ", ";
            }std::cout << endl;
            ur5->moveJP(target_pose_2);

            for (auto element : target_pose_3)
            {
                cout << element << ", ";
            }std::cout << endl;
            ur5->moveJP(target_pose_3);
        }
    }
    
    emit shelf_grasp_finished();
    }
    catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        emit shelf_grasp_finished();
    }
}

cv::Mat Function::getRotationMatrixAroundAxis(double x, double y, double z, double angle)
{
    // 旋转轴向量
    cv::Vec3d axis(x, y, z);
    cv::normalize(axis, axis);

    double angleRadians = angle * CV_PI / 180.0;

    // 创建旋转向量
    cv::Vec3d rvec = axis * angleRadians;

    // 将旋转向量转换为旋转矩阵
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    // 创建齐次变换矩阵
    cv::Mat RT = cv::Mat::eye(4, 4, CV_64F);
    R.copyTo(RT(cv::Rect(0, 0, 3, 3)));
    
    return RT.clone();
}

// 函数计算点p绕点center旋转theta度后的新坐标
cv::Point2d Function::rotatePoint(cv::Point2d p, cv::Point2d center, double theta) {
    // 将角度从度转换为弧度
    double rad = theta * M_PI / 180.0;

    // 平移到旋转中心
    double px = p.x - center.x;
    double py = p.y - center.y;

    // 旋转点
    double newX = px * cos(rad) - py * sin(rad);
    double newY = px * sin(rad) + py * cos(rad);

    // 平移回原来的位置
    cv::Point2d newPoint(newX + center.x, newY + center.y);
    return newPoint;
}

