#include <UR5.h>
#include <iostream>


UR5::UR5(string robot_ip, bool use_camera) \
    : robot_ip(robot_ip), control_socket(io_service), use_camera(use_camera), q_actual(6), tool_vector_actual(6), receive_buffer(1220), \
    T_End2Base(3, 1, CV_64F), rotation_vector(3, 1, CV_64F) {
}

void UR5::connect() {
    try
    {
        this->control_socket.connect(tcp::endpoint(boost::asio::ip::address::from_string(robot_ip), control_port));
    }
    catch (exception& e) {
        cerr << "Exception: " << e.what() << endl;
    }
    cout << "Robot connected" << endl;   
}

    
void UR5::disconnect() {
    try
    {
        this->control_socket.close();
    }
    catch (exception& e) {
        cerr << "Exception: " << e.what() << endl;
    }
    cout << "Robot disconnected" << endl; 
}

void UR5::sendCommand(tcp::socket& socket, const string command) {
    boost::asio::write(socket, boost::asio::buffer(command));
}

tuple<vector<double>, vector<double>> UR5::getCurrentPositions() {
    boost::asio::io_service io_service;
    tcp::socket read_socket(io_service);
    read_socket.connect(tcp::endpoint(boost::asio::ip::address::from_string(robot_ip), read_port));

    // string tcp_command = "get_actual_joint_positions()\n";
    // sendCommand(read_socket, tcp_command);

    this->receive_len = read_socket.read_some(boost::asio::buffer(this->receive_buffer.data(), this->receive_buffer.size()));

    for (int i = 0; i < 6; i++)
    {
        reverse_copy(this->receive_buffer.begin() + this->offset_q + i*sizeof(double), \
        this->receive_buffer.begin() + this->offset_q + (i + 1)*sizeof(double), reinterpret_cast<char*>(&this->q_actual[i]));
        reverse_copy(this->receive_buffer.begin() + this->offset_tool + i*sizeof(double), \
        this->receive_buffer.begin() + this->offset_tool + (i + 1)*sizeof(double), reinterpret_cast<char*>(&this->tool_vector_actual[i]));
    }


    // cout << "Joint: ";
    // for (double angle : this->q_actual)
    // {
    //     cout << angle << " ";
    // }
    // cout << endl;

    cout << "Pose: ";
    for (double pose : this->tool_vector_actual)
    {
        cout << pose << ", ";
    }
    cout << endl << endl;    

    read_socket.close();
    return make_tuple(this->q_actual, this->tool_vector_actual);   
}

cv::Mat UR5::eulerAnglesToRotationMatrix(const cv::Mat_<double>& eulerAngles)
{
    // Calculate rotation about x axis
    cv::Mat R_x = (cv::Mat_<double>(3,3) <<
        1,       0,              0,
        0,       cos(eulerAngles(0)),   -sin(eulerAngles(0)),
        0,       sin(eulerAngles(0)),   cos(eulerAngles(0))
    );
    // Calculate rotation about y axis
    cv::Mat R_y = (cv::Mat_<double>(3,3) <<
        cos(eulerAngles(1)),    0,      sin(eulerAngles(1)),
        0,               1,      0,
        -sin(eulerAngles(1)),   0,      cos(eulerAngles(1))
    );
    // Calculate rotation about z axis
    cv::Mat R_z = (cv::Mat_<double>(3,3) <<
        cos(eulerAngles(2)),    -sin(eulerAngles(2)),      0,
        sin(eulerAngles(2)),    cos(eulerAngles(2)),       0,
        0,               0,                  1
    );
    // Combined rotation matrix
    cv::Mat R = R_z * R_y * R_x;
    // cv::Mat R = R_x * R_y * R_z;
    return R;
}

cv::Mat UR5::R_and_T2RT(cv::Mat R, cv::Mat T) {
    cv::Mat RT = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat R_part = RT(cv::Rect(0, 0, 3, 3));
    cv::Mat T_part = RT.col(3).rowRange(0, 3);
    R.copyTo(R_part);
    T.copyTo(T_part);

    return RT;
}

tuple<cv::Mat, cv::Mat> UR5::getCurrentRT(){
    boost::asio::io_service io_service;
    tcp::socket read_socket(io_service);
    read_socket.connect(tcp::endpoint(boost::asio::ip::address::from_string(robot_ip), read_port));

    this->receive_len = read_socket.read_some(boost::asio::buffer(this->receive_buffer.data(), this->receive_buffer.size()));

    for (int i = 0; i < 6; i++)
    {
        reverse_copy(this->receive_buffer.begin() + this->offset_q + i*sizeof(double), \
        this->receive_buffer.begin() + this->offset_q + (i + 1)*sizeof(double), reinterpret_cast<char*>(&this->q_actual[i]));
        reverse_copy(this->receive_buffer.begin() + this->offset_tool + i*sizeof(double), \
        this->receive_buffer.begin() + this->offset_tool + (i + 1)*sizeof(double), reinterpret_cast<char*>(&this->tool_vector_actual[i]));
    }

    for (size_t i = 0; i < 3; i++)
    {
        this->T_End2Base.at<double>(i) = this->tool_vector_actual[i];
        this->rotation_vector.at<double>(i) = this->tool_vector_actual[i + 3];
    }
    // this->R_End2Base = eulerAnglesToRotationMatrix(this->rotation_vector);
    cv::Rodrigues(this->rotation_vector, this->R_End2Base);

    // cout << "T_End2Base" << T_End2Base << endl;
    // cout << "rotation_vector: " << rotation_vector << endl;
    // cout << "R_End2Base: " << R_End2Base << endl;
   
    return make_tuple(this->R_End2Base.clone(), this->T_End2Base.clone());
}

void UR5::moveJ(const vector<double>& joint_configuration) {
    // 构建 URScript 命令
    string tcp_command = "movej([";
    for (size_t i = 0; i < joint_configuration.size(); ++i) {
        tcp_command += to_string(joint_configuration[i]);
        if (i < joint_configuration.size() - 1) tcp_command += ",";
    }
    tcp_command += "],a=" + to_string(this->joint_acc) + ",v=" + to_string(this->joint_vel) + ",t=" + to_string(this->t) + ",r=" + to_string(this->r) + ")\n";
    sendCommand(this->control_socket, tcp_command);
    this_thread::sleep_for(chrono::milliseconds(500));

    getCurrentPositions();
    
    
    while (true)
    {
        
        bool position_reached = true;
        for (size_t i = 0; i < 6; i++)
        {
            if (abs(this->q_actual[i] - joint_configuration[i]) > this->joint_tolerance)
            {
                position_reached = false;
                break;
            }
        }
        if (position_reached)
            {
                break;
            }  
        this_thread::sleep_for(chrono::milliseconds(4000));
        getCurrentPositions();
        
    }
    cout << "Robot arrived" << endl;
    this_thread::sleep_for(chrono::milliseconds(500));

    
}

void UR5::moveJP(const vector<double>& tool_configuration) {
    // string tcp_command = "def process():\n";
    // tcp_command += " array = rpy2rotvec([" + to_string(tool_configuration[3]) + "," + to_string(tool_configuration[4]) + "," + to_string(tool_configuration[5]) + "])\n";
    // tcp_command += "movej(get_inverse_kin(p[" + to_string(tool_configuration[0]) + "," + to_string(tool_configuration[1]) + "," + to_string(tool_configuration[2]) \
    // + ",array[0],array[1],array[2]]),a=" + to_string(this->joint_acc) + ",v=" + to_string(this->joint_vel) + ",t=" + to_string(this->t) + ",r=" + to_string(this->r) + ")\n";
    // tcp_command += "end\n";
    string tcp_command = "movej(get_inverse_kin(p[";
    for (size_t i = 0; i < tool_configuration.size(); ++i) {
        tcp_command += to_string(tool_configuration[i]);
        if (i < tool_configuration.size() - 1) tcp_command += ",";
    }
    tcp_command += "]),a=" + to_string(this->joint_acc) + ",v=" + to_string(this->joint_vel) + ",t=" + to_string(this->t) + ",r=" + to_string(this->r) + ")\n";
    
    cout << tcp_command << endl; // 输出一下看看
    sendCommand(this->control_socket, tcp_command);
    this_thread::sleep_for(chrono::milliseconds(500));

    // 接收机械臂返回的数据
    getCurrentPositions();
    
    while (true)
    {
        
        bool position_reached = true;
        for (size_t i = 0; i < 6; i++)
        {
            if (abs(this->tool_vector_actual[i] - tool_configuration[i]) > this->tool_tolerance[i])
            {
                position_reached = false;
                break;
            }
        }
        if (position_reached)
            {
                break;
            }  
        this_thread::sleep_for(chrono::milliseconds(4000));
        getCurrentPositions();
        
    }
    cout << "Robot arrived" << endl;
    this_thread::sleep_for(chrono::milliseconds(500));
    
}

void UR5::moveJP(const vector<double>& tool_configuration, int mode)
{
    string tcp_command = "movej(get_inverse_kin(p[";
    for (size_t i = 0; i < tool_configuration.size(); ++i) {
        tcp_command += to_string(tool_configuration[i]);
        if (i < tool_configuration.size() - 1) tcp_command += ",";
    }
    tcp_command += "]),a=" + to_string(this->joint_acc) + ",v=" + to_string(this->joint_vel) + ",t=" + to_string(this->t) + ",r=" + to_string(this->r) + ")\n";
    
    cout << tcp_command << endl; // 输出一下看看
    sendCommand(this->control_socket, tcp_command);

    // // 接收机械臂返回的数据
    // getCurrentPositions();
    
    // while (true)
    // {
        
    //     bool position_reached = true;
    //     for (size_t i = 0; i < 6; i++)
    //     {
    //         if (abs(this->tool_vector_actual[i] - tool_configuration[i]) > this->tool_tolerance[i])
    //         {
    //             position_reached = false;
    //             break;
    //         }
    //     }
    //     if (position_reached)
    //         {
    //             break;
    //         }  
    //     this_thread::sleep_for(chrono::milliseconds(5));
    //     getCurrentPositions();
        
    // }
    // cout << "Robot arrived" << endl;
    // this_thread::sleep_for(chrono::milliseconds(10));
}

void UR5::moveL(const vector<double>& tool_configuration) {
    // 构建 URScript 命令
    string tcp_command = "movel(p[";
    for (size_t i = 0; i < tool_configuration.size(); ++i) {
        tcp_command += to_string(tool_configuration[i]);
        if (i < tool_configuration.size() - 1) tcp_command += ",";
    }
    tcp_command += "],a=" + to_string(this->joint_acc) + ",v=" + to_string(this->joint_vel) + ",t=" + to_string(this->t) + ",r=" + to_string(this->r) + ")\n";
    sendCommand(this->control_socket, tcp_command);
    this_thread::sleep_for(chrono::milliseconds(500));

    getCurrentPositions();
    
    
    while (true)
    {
        
        bool position_reached = true;
        for (size_t i = 0; i < 6; i++)
        {
            if (abs(this->tool_vector_actual[i] - tool_configuration[i]) > this->tool_tolerance[i])
            {
                position_reached = false;
                break;
            }
        }
        if (position_reached)
            {
                break;
            }  
        this_thread::sleep_for(chrono::milliseconds(4000));
        getCurrentPositions();
        
    }
    cout << "Robot arrived" << endl;
    this_thread::sleep_for(chrono::milliseconds(500));

    
}