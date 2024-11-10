#ifndef UR5_H
#define UR5_H

#include <boost/asio.hpp>
#include <opencv2/opencv.hpp>

using boost::asio::ip::tcp;
using namespace std;
using namespace cv;

const string robot_ip = "192.168.50.20";
const unsigned short read_port = 30003;
const unsigned short control_port = 30002;


class UR5
{
private:
    string robot_ip;
    int read_port = 30003;
    int control_port = 30001;
    boost::asio::io_service io_service;
    tcp::socket control_socket;
    bool use_camera;

    double joint_acc = 0.2;
    double joint_vel = 0.2;
    double t = 0.0;
    double r = 0.0;
    double joint_tolerance = 0.01;
    double tool_tolerance[6] = {0.002, 0.002, 0.002, 0.01, 0.01, 0.01};
    int offset_q = 4 + 8 + 48 * 5;
    int offset_tool = 4 + 8 + 48 * 9;
    int offset_number = 0;
    vector<double> q_actual;
    vector<double> tool_vector_actual;
    vector<char> receive_buffer;
    size_t receive_len;
    cv::Mat R_End2Base, rotation_vector, T_End2Base;
    

public:
    UR5(string robot_ip = "192.168.50.20", bool use_camera = true);

    void connect();
    void disconnect();
    void sendCommand(tcp::socket& socket, const string command);
    tuple<vector<double>, vector<double>> getCurrentPositions();
    cv::Mat eulerAnglesToRotationMatrix(const cv::Mat_<double>& eulerAngles);
    tuple<cv::Mat, cv::Mat> getCurrentRT();
    cv::Mat R_and_T2RT(cv::Mat R, cv::Mat T);
    void moveJ(const vector<double>& joint_configuration);
    void moveJP(const vector<double>& tool_configuration);
    void moveJP(const vector<double>& tool_configuration, int mode);
    void moveL(const vector<double>& tool_configuration);
};


#endif