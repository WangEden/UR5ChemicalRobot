#include "calibrate.h"
// #include "ur_client_library/rtde/rtde_client.h"
#include <iostream>
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <Eigen/Dense>

tuple<cv::Mat, cv::Mat> calibrate()
{
    vector<double> xyz_base = {-0.262, -0.640, 0.294};
    vector<vector<double>> rotation_base = {{-1.704, -2.378, -0.299},
                                            {-2.443, -1.898, 0.051},
                                            {-2.358, -2.027, 0.042},
                                            {1.652, 2.392, -0.256},
                                            {1.765, 2.015, -0.561}};
    vector<double> pose;

    vector<double> ini_joints = {-1.478, -1.591, -1.574, -1.544, 1.565, 0.077};
    vector<double> init_pose4 = {-0.113749, -0.42797, 0.356629, -3.13985, 0.102819, -6.66636e-08};
    //   vector<vector<double>> test_pose = {
    //       {-0.088, -0.511, 0.481, 0.023, -3.130, 0.010},
    //       {-0.080, -0.372, 0.470, 0.769, 2.827, -0.577},
    //       {-0.078, -0.521, 0.613, 0.049, -3.127, 0.021},
    //       {-0.099, -0.512, 0.707, 0.032, 3.113, 0.034},
    //       {-0.037, -0.321, 0.464, 1.248, 2.654, -0.592},
    //       {-0.155, -0.411, 0.300, -0.179, 3.018, -0.458},
    //       {-0.217, -0.395, 0.416, -0.670, 2.890, -0.638},
    //       {-0.349, -0.429, 0.368, -0.115, 2.517, -0.348},
    //       {-0.132, -0.249, 0.305, -0.156, 2.776, -1.323},
    //       {-0.304, -0.382, 0.367, -0.733, 2.659, -0.938},
    //       {-0.314, -0.329, 0.405, -0.698, 2.591, -0.748},
    //       {-0.139, -0.500, 0.488, -0.202, 3.102, -0.172},
    //       {-0.295, -0.494, 0.367, -0.081, 2.603, -0.177},
    //       {-0.271, -0.460, 0.524, -0.973, 2.669, -0.411},
    //       {0.151, -0.382, 0.545, -0.849, -2.660, 0.353},
    //       {0.118, -0.535, 0.403, 0.028, -2.581, 0.000},
    //       {0.155, -0.652, 0.328, -0.022, -2.561, -0.443},
    //       {0.167, -0.649, 0.442, 0.536, -2.681, -0.325},
    //       {-0.113, -0.692, 0.428, 0.959, 2.831, 0.654},
    //       {-0.118, -0.647, 0.417, -0.839, -2.951, -0.532},
    //       {-0.108, -0.731, 0.347, -0.396, 2.936, 0.687},
    //       {-0.078, -0.771, 0.339, -0.725, 2.814, 0.719},
    //       {-0.250, -0.726, 0.391, 0.260, 2.772, 0.583},
    //       {-0.325, -0.698, 0.390, 0.177, 2.667, 0.452}};
    //   vector<vector<double>> test_pose = {
    //       {-0.0880094, -0.511011, 0.480979, 1.19257, -2.88205, 0.0395662},
    //       {-0.0804625, -0.273681, 0.474247, 1.16608, -2.69736, 0.64359},
    //       {-0.078, -0.521, 0.613, 0.049, -3.127, 0.021},
    //       {-0.099, -0.512, 0.707, 0.032, 3.113, 0.034},
    //       {-0.0372065, -0.320181, 0.461971, 1.14601, -2.66265, 0.530848},
    //       {-0.158756, -0.401181, 0.422821, -0.178949, 3.01802, -0.457991},
    //       {-0.217, -0.395, 0.416, -0.670, 2.890, -0.638},
    //       {-0.352997, -0.435659, 0.376118, -0.154852, 2.54951, -0.444779},
    //       {-0.126988, -0.2493, 0.295658, 1.10157, -2.50279, 1.17669},
    //       {-0.297154, -0.374823, 0.34832, -0.638769, 2.58995, -0.761685},
    //       {-0.314, -0.329, 0.405, -0.698, 2.591, -0.748},
    //       {-0.139, -0.500, 0.488, -0.202, 3.102, -0.172},
    //       {-0.297154, -0.374823, 0.34832, -0.638769, 2.58995, -0.761685},
    //       {-0.271, -0.460, 0.524, -0.973, 2.669, -0.411},
    //       {0.151, -0.382, 0.545, -0.849, -2.660, 0.353},
    //       {0.128087, -0.541134, 0.407751, -0.0115717, -2.68044, 0.0675476},
    //       {0.151964, -0.662609, 0.332209, -0.0748618, -2.5287, -0.30324},
    //       {0.167, -0.649, 0.442, 0.536, -2.681, -0.325},
    //       {-0.134228, -0.6911, 0.425994, -0.892859, -2.83319, -0.554866},
    //       {-0.127428, -0.646681, 0.416649, -0.792911, -2.8775, -0.482397},
    //       {-0.108548, -0.738003, 0.349519, -0.403814, 2.96481, 0.612656},
    //       {-0.078, -0.771, 0.339, -0.725, 2.814, 0.719},
    //       {-0.250, -0.726, 0.391, 0.260, 2.772, 0.583},
    //       {-0.325, -0.698, 0.390, 0.177, 2.667, 0.452}};
    vector<vector<double>> test_pose = {
        {-0.145686, -0.661804, 0.223305, 0.015118, 2.86222, 0.411914},
        {-0.144888, -0.546389, 0.428986, -0.105863, 2.98335, -0.0525052},
        {0.0355672, -0.634795, 0.272219, 0.807563, -2.7929, -0.349001},
        {0.0380452, -0.641726, 0.138368, 0.819413, -2.70957, -0.693157},
        {-0.0419308, -0.670119, 0.116147, 0.546952, -2.90765, -0.838728},
        {-0.169842, -0.691647, 0.188383, 0.806346, 2.84445, 0.747452},
        {-0.223738, -0.522365, 0.262737, 0.942515, 2.76486, -0.0773243},
        {-0.271829, -0.490522, 0.271685, 1.22917, 2.47961, -0.0747715},
        {-0.265125, -0.408735, 0.355462, 1.17126, 2.48985, -0.367735},
        {-0.100878, -0.322317, 0.348836, 2.58433, -1.43653, 0.235336},
        {-0.100735, -0.342867, 0.343796, 2.88565, 0.621439, -0.395578},
        {-0.0764124, -0.454887, 0.234642, -2.77783, -0.700619, 0.579378},
        {-0.0232448, -0.600678, 0.274501, -2.40186, -0.707487, 0.502737},
        {-0.0311692, -0.570619, 0.26512, -2.55157, 0.291859, 0.294494},
        {-0.072469, -0.53573, 0.257607, -2.59735, 0.71417, 0.252685},
        {-0.0807206, -0.54964, 0.299613, -2.51502, 0.702905, 0.239865},
        {-0.0133773, -0.578725, 0.324571, -2.34917, -1.00701, 0.498883},
        {0.0176341, -0.432528, 0.312408, -2.9983, -0.117498, 0.5544},
        {0.0176341, -0.432528, 0.312408, -2.9983, -0.117498, 0.5544},
        {-0.114467, -0.374808, 0.202832, -3.08088, -0.355197, 0.436559},
        {-0.172179, -0.376838, 0.230468, 3.08868, 0.361486, -0.0818288},
        {-0.346649, -0.414781, 0.104354, 2.89661, 0.476665, 1.1003},
        {-0.346649, -0.414781, 0.104354, 2.89661, 0.476665, 1.1003},
        {-0.0950828, -0.320464, 0.235713, 2.83027, 0.11759, -0.323062}
    };

    vector<cv::Mat> R_End2Base, T_End2Base, R_Board2Cam, T_Board2Cam;
    cv::Mat R_Cam2End, T_Cam2End;
    tuple<cv::Mat, cv::Mat> RT_Board2Cam, RT_End2Base;

    UR5 ur5;
    Camera camera;

    camera.connect();
    ur5.connect();
    ur5.moveJP(init_pose4);
    //   ur5.moveJ(ini_joints);

    bool save = true;
    bool projection = false;

    for (vector<double> pose : test_pose)
    {
        ur5.moveJP(pose);
        // ur5.getCurrentPositions();
        RT_End2Base = ur5.getCurrentRT();
        R_End2Base.push_back(get<0>(RT_End2Base));
        T_End2Base.push_back(get<1>(RT_End2Base));
        // std::cout << "R_End2Base: " << R1 << endl;
        // std::cout << "T_End2Base: " << T1 << endl << endl;

        camera.get_images();
        // imshow("frame", get<0>(mat_tuple));
        // waitKey(0);
        camera.get_corners(save);
        RT_Board2Cam = camera.get_R_board2cam(projection);

        cv::Mat R2 = get<0>(RT_Board2Cam);
        cv::Mat T2 = get<1>(RT_Board2Cam);
        R_Board2Cam.push_back(R2);
        T_Board2Cam.push_back(T2);
        // cout << "R_Board2Cam: " << R2 << endl;
        // cout << "T_End2Base: " << T2 << endl << endl;
    }

    // for (cv::Mat T : T_End2Base)
    // {
    //     cout << "T: " << T << endl;
    // }

    cout << R_End2Base.size() << " " << T_End2Base.size() << " "
         << R_Board2Cam.size() << " " << T_Board2Cam.size() << endl;
    cv::calibrateHandEye(R_End2Base, T_End2Base, R_Board2Cam, T_Board2Cam,
                         R_Cam2End, T_Cam2End);
    cout << R_Cam2End << endl;
    cout << T_Cam2End << endl;
    cv::FileStorage file("../data/Cam2End.yml", cv::FileStorage::WRITE);
    if (file.isOpened())
    {
        file << "R_Cam2End" << R_Cam2End;
        file << "T_Cam2End" << T_Cam2End;
        file.release();
    }

    ur5.disconnect();
    camera.disconnect();

    return make_tuple(R_Cam2End, T_Cam2End);
}

using boost::asio::ip::tcp;

const std::string ROBOT_IP = "192.168.50.20"; // 机器人的IP地址
const int PORT = 30004; // RTDE端口



char getch() {
    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0)
        perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0)
        perror ("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
        perror ("tcsetattr ~ICANON");
    printf("%c\n", buf);
    return buf;
}

void sendControlCommand(tcp::socket &socket, const Eigen::VectorXd &command) {
    boost::system::error_code ignored_error;
    boost::asio::write(socket, boost::asio::buffer(command.data(), command.size() * sizeof(double)), ignored_error);
}

void controlRobot(tcp::socket &socket) {
    Eigen::VectorXd control_command(6);
    control_command.setZero();

    while (true) {
        char ch = getch();
        switch (ch) {
            case 'w':
                control_command[1] += 0.01; // 向前移动
                break;
            case 's':
                control_command[1] -= 0.01; // 向后移动
                break;
            case 'a':
                control_command[0] -= 0.01; // 向左移动
                break;
            case 'd':
                control_command[0] += 0.01; // 向右移动
                break;
            case 'q':
                control_command[2] += 0.01; // 向上移动
                break;
            case 'e':
                control_command[2] -= 0.01; // 向下移动
                break;
            case 'x':
                std::cout << "Exiting control loop." << std::endl;
                return;
            default:
                break;
        }
        // std::cout << "Control Command: " << control_command.transpose() << std::endl;
        // sendControlCommand(socket, control_command);

    }
}

int main()
{
    cv::Mat markerImage;
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::generateImageMarker(dictionary, 23, 400, markerImage, 1);
    cv::imwrite("../data/imgs/marker23.png", markerImage);

    cv::Mat inputImage;
    // ... read inputImage ...
    inputImage = cv::imread("../data/imgs/123.jpg", cv::IMREAD_COLOR);

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);
    detector.detectMarkers(inputImage, markerCorners, markerIds, rejectedCandidates);

    std::cout << "Detected " << markerIds.size() << " markers." << std::endl;

    cv::Mat outputImage = inputImage.clone();
    cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

    for (int i = 0; i < markerIds.size(); i++) {
        std::cout << "Marker ID: " << markerIds[i] << std::endl;
        for (int j = 0; j < 4; j++) {
            std::cout << "Corner " << j << ": " << markerCorners[i][j] << std::endl;
            cv::circle(outputImage, markerCorners[i][j], 5, cv::Scalar(0, 0, 255), -1);
            cv::putText(outputImage, std::to_string(j+1), markerCorners[i][j], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        }
    }

    cv::imwrite("../data/imgs/output.jpg", outputImage);

    return 0;

    // try {
    //     boost::asio::io_service io_service;
    //     tcp::resolver resolver(io_service);
    //     tcp::resolver::query query(ROBOT_IP, std::to_string(PORT));
    //     tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);

    //     tcp::socket socket(io_service);
    //     boost::asio::connect(socket, endpoint_iterator);

    //     std::thread control_thread(controlRobot, std::ref(socket));
    //     control_thread.join();
    // } catch (std::exception &e) {
    //     std::cerr << "Exception: " << e.what() << std::endl;
    // }

    // return 0;


    //   RTDEClient(std::string robot_ip, comm::INotifier& notifier, const std::string& output_recipe_file,
    //  const std::string& input_recipe_file, double target_frequency = 0.0);
    // urcl::rtde_interface::RTDEClient rtde_client(""


    // calibrate();
    // vector<vector<double>> test_pose = {
    //     // {-0.145686, -0.661804, 0.223305, 0.015118, 2.86222, 0.411914},
    //     // {-0.144888, -0.546389, 0.428986, -0.105863, 2.98335, -0.0525052},
    //     // {0.0355672, -0.634795, 0.272219, 0.807563, -2.7929, -0.349001},
    //     // {0.0380452, -0.641726, 0.138368, 0.819413, -2.70957, -0.693157},
    //     // {-0.0419308, -0.670119, 0.116147, 0.546952, -2.90765, -0.838728},
    //     // {-0.169842, -0.691647, 0.188383, 0.806346, 2.84445, 0.747452},
    //     // {-0.223738, -0.522365, 0.262737, 0.942515, 2.76486, -0.0773243},
    //     // {-0.271829, -0.490522, 0.271685, 1.22917, 2.47961, -0.0747715},
    //     // {-0.265125, -0.408735, 0.355462, 1.17126, 2.48985, -0.367735},
    //     // {-0.100878, -0.322317, 0.348836, 2.58433, -1.43653, 0.235336},
    //     // {-0.100735, -0.342867, 0.343796, 2.88565, 0.621439, -0.395578},
    //     // {-0.0764124, -0.454887, 0.234642, -2.77783, -0.700619, 0.579378},
    //     // {-0.0232448, -0.600678, 0.274501, -2.40186, -0.707487, 0.502737},
    //     // {-0.0311692, -0.570619, 0.26512, -2.55157, 0.291859, 0.294494},
    //     // {-0.072469, -0.53573, 0.257607, -2.59735, 0.71417, 0.252685},
    //     // {-0.0807206, -0.54964, 0.299613, -2.51502, 0.702905, 0.239865},
    //     // {-0.0133773, -0.578725, 0.324571, -2.34917, -1.00701, 0.498883},
    //     // {0.0176341, -0.432528, 0.312408, -2.9983, -0.117498, 0.5544},
    //     // {0.0176341, -0.432528, 0.312408, -2.9983, -0.117498, 0.5544},
    //     // {-0.114467, -0.374808, 0.202832, -3.08088, -0.355197, 0.436559},
    //     // {-0.172179, -0.376838, 0.230468, 3.08868, 0.361486, -0.0818288},
    //     // {-0.346649, -0.414781, 0.104354, 2.89661, 0.476665, 1.1003},
    //     // {-0.346649, -0.414781, 0.104354, 2.89661, 0.476665, 1.1003},
    //     // {-0.0950828, -0.320464, 0.235713, 2.83027, 0.11759, -0.323062}
    // };

    // UR5 ur5;
    // Camera camera;
    // ur5.connect();
    // camera.connect();

    // // for (vector<double> pose : test_pose)
    // // {
    // //     ur5.moveJP(pose);
    // //     auto mat_tuple = camera.get_images();
    // //     auto color = std::get<0>(mat_tuple);
    // //     cv::imshow("frame", color);
    // //     cv::waitKey(0);
    // // };
    // ur5.moveJP(test_pose[0]);
    // while (true)
    // {
    //     auto mat_tuple = camera.get_images();
    //     auto color = std::get<0>(mat_tuple);
    //     cv::imshow("frame", color);
    //     int key = cv::waitKey(1);
    //     if (key == 27)
    //     {
    //         break;
    //     }
    // }
    // ur5.getCurrentPositions();

    // ur5.disconnect();
    // camera.disconnect();

    // return 0;
}
