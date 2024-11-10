#include "widget.h"
#include <qpushbutton.h>
#include <vector>
#include <QDebug>
#include <sstream>

#include "d435i.h"
#include "UR5.h"

#define RS485

Widget::Widget(QWidget *parent)
    : QWidget(parent)
{
    qDebug() << "主线程：" << QThread::currentThreadId();
    // 设置窗口属性
    // 读取配置文件
    cv::FileStorage fs("/home/eden/Project/AllFunc/app_ew820/app_ew/configs/config.yml", cv::FileStorage::READ);
    fs["width"] >> this->WIDTH;
    fs["height"] >> this->HEIGHT;
    fs.release();
    resize(this->WIDTH, this->HEIGHT);
    this->setWindowTitle("识别抓取实验");

    // 创建控件
    drawWidget();

    // 初始化复选框内容
    initComboBox();

    // 从复选框中获取信息
    getBaudrate(this->boxBaudrate->currentIndex());
    getDataBits(this->boxDatabits->currentIndex());
    getStopbits(this->boxDatabits->currentIndex());
    getParity(this->boxParitybits->currentIndex());

    // 扫描一次串口设备
#ifndef RS485
    this->serialport = new SerialPort();
    this->boxPortName->addItems(this->serialport->scanPortName());
#else
    this->rs485 = new RS485ModbusRtuMaster(this);
    this->boxPortName->addItems(this->rs485->scanPortName());
#endif
    // 初始化串口设备
    QString portname = this->boxPortName->currentText();
#ifndef RS485
    this->serialport->serialPortInit(
        portname, 
        this->baudrate, 
        this->dataBits, 
        this->parity, 
        this->stopbits);
#else
    this->rs485->setPortName(portname);
    this->rs485->setBaudRate(this->baudrate);
    this->rs485->setDataBits(this->dataBits);
    this->rs485->setParity(this->parity);
    this->rs485->setStopBits(this->stopbits);
#endif
    // 初始化定时器
    initQTimer();

    // 初始化信号槽
    initConnection();

    this->ur5 = new UR5();
    // this->ur5->connect();

    this->function = new Function();
    this->function2 = new Function(); 
    connect(function2, &Function::init_pose_change, function, &Function::change_init_pose);
}

Widget::~Widget() 
{
    if (this->serialport != nullptr) {
        delete this->serialport;
    }
    this->ur5->disconnect();
}

void Widget::switchSerialPortStatus()
{
    // 打开串口的按钮按下时触发
    if (!this->btnPortSwitch->text().compare("打开串口")) {
        this->btnPortSwitch->setText("关闭串口");

        // 使复选框失能
        this->boxPortName->setEnabled(false);
        this->boxBaudrate->setEnabled(false);
        this->boxDatabits->setEnabled(false);
        this->boxStopbits->setEnabled(false);
        this->boxParitybits->setEnabled(false);
        this->boxProtocol->setEnabled(false);

        // 初始化串口设备
        QString portname = this->boxPortName->currentText();
#ifndef RS485
        this->serialport->serialPortInit(
            portname, 
            this->baudrate, 
            this->dataBits, 
            this->parity, 
            this->stopbits);
        this->serialport->openPort(); // 打开串口
#else
        this->rs485->setPortName(portname);
        this->rs485->setBaudRate(this->baudrate);
        this->rs485->setDataBits(this->dataBits);
        this->rs485->setParity(this->parity);
        this->rs485->setStopBits(this->stopbits);
        this->rs485->openPort();       
#endif 
    } 
    else {
        this->btnPortSwitch->setText("打开串口");
        // 使复选框使能
        this->boxPortName->setEnabled(true);
        this->boxBaudrate->setEnabled(true);
        this->boxDatabits->setEnabled(true);
        this->boxStopbits->setEnabled(true);
        this->boxParitybits->setEnabled(true);
        this->boxProtocol->setEnabled(true);
#ifndef RS485
        // 删除串口设备对象
        this->serialport->closePort(); // 关闭串口
#else
        this->rs485->closePort();
#endif 
    }
}

void Widget::flashSerialPortDevice()
{
    // 扫描串口设备
    this->boxPortName->clear();   
    this->boxPortName->addItems(serialport->scanPortName());
}

void Widget::clearMessage()
{
    this->textBrowser->clear();
}

void Widget::sendMessage()
{
    // 获取lineEdit中的内容
    QString data = this->lineEdit->text();
    // 发送
    // 发送字符串
    // this->serialport->sendMessage(data);
    // 发送字节流
    QByteArray sendData;


    // 假设这是要转换的字符串
    // QString hexString = "14 5A 6E 87 4F";
    // 使用空白字符分割字符串
    QStringList hexList = data.split(" ");

    QByteArray byteArray;
    bool ok;
    for (const QString &hex : hexList) {
        // 将十六进制字符串转换为字节
        byteArray.append(static_cast<char>(hex.toInt(&ok, 16)));
    }
#ifndef RS485
    this->serialport->sendMessage(byteArray);
#else
    this->rs485->sendCommand(byteArray);
#endif
    // 清空编辑框
    clearEdit();
    this->textBrowser->append("<span style='color: red;'>Send: " +data + "</span>");
}

void Widget::getBaudrate(int index)
{
    // QStringList list1 = {"9600", "19200", "38400", "57600", "115200"};
    std::vector<QSerialPort::BaudRate> l = {
        QSerialPort::Baud9600, 
        QSerialPort::Baud19200,
        QSerialPort::Baud38400,
        QSerialPort::Baud57600,
        QSerialPort::Baud115200
    };
    this->baudrate = l.at(this->boxBaudrate->currentIndex());
    // qDebug() << "当前波特率:" << this->baudrate;
}

void Widget::getDataBits(int index)
{
    std::vector<QSerialPort::DataBits> l = {
        QSerialPort::Data8, 
        QSerialPort::Data7,
        QSerialPort::Data6,
        QSerialPort::Data5,
    };
    this->dataBits = l.at(this->boxDatabits->currentIndex());
}

void Widget::getParity(int index)
{
    std::vector<QSerialPort::Parity> l = {
        QSerialPort::NoParity, 
        QSerialPort::OddParity,
        QSerialPort::EvenParity
    };
    this->parity = l.at(this->boxParitybits->currentIndex());
}

void Widget::getStopbits(int index)
{
    std::vector<QSerialPort::StopBits> l = {
        QSerialPort::UnknownStopBits, 
        QSerialPort::OneStop,
        QSerialPort::TwoStop
    };
    this->stopbits = l.at(this->boxStopbits->currentIndex());
}

void Widget::clearEdit()
{
    this->lineEdit->setText("");
}

void Widget::showMessage()
{
#ifndef RS485
    QByteArray data = this->serialport->getQSerialPort()->readAll();
    this->textBrowser->append("Recv: "+data);
#else
    QByteArray data = this->rs485->getTheResponseVariable();
    this->textBrowser->append("Recv: "+QString::fromUtf8(data.toHex()));
#endif
}

void Widget::Utils::splitString(const std::string & in_, std::vector<std::string> & out, char split_ = ' ')
{
    std::istringstream iss(in_);
    std::string token;
    while (std::getline(iss, token, split_)) {
        out.push_back(token);
    }
}

void Widget::drawWidget()
{
    // 图像显示框
    this->cameraWidget = new CameraWidget(this); // 上左
    this->cameraWidget->setGeometry(int(this->WIDTH * 0.03), int(this->HEIGHT * 0.05),
                            int(this->WIDTH * 0.45), int(this->HEIGHT * 0.40));

    // 文本显示框
    this->textBrowser = new QTextBrowser(this); // 上右
    this->textBrowser->setGeometry(int(this->WIDTH * 0.52), int(this->HEIGHT * 0.05),
                             int(this->WIDTH * 0.45), int(this->HEIGHT * 0.30));

    // 机械臂微调按钮
    this->btnUR5Front = new QPushButton("机械臂前移", this);
    this->btnUR5Front->setGeometry(int(this->WIDTH * 0.6825), int(this->HEIGHT * 0.35), int(this->WIDTH * 0.13), int(this->HEIGHT * 0.05));

    this->btnUR5Back = new QPushButton("机械臂后移", this);
    this->btnUR5Back->setGeometry(int(this->WIDTH * 0.6825), int(this->HEIGHT * 0.40), int(this->WIDTH * 0.13), int(this->HEIGHT * 0.05));

    this->btnUR5Left = new QPushButton("机械臂左移", this);
    this->btnUR5Left->setGeometry(int(this->WIDTH * 0.52), int(this->HEIGHT * 0.40), int(this->WIDTH * 0.13), int(this->HEIGHT * 0.05));

    this->btnUR5Right = new QPushButton("机械臂右移", this);
    this->btnUR5Right->setGeometry(int(this->WIDTH * 0.84), int(this->HEIGHT * 0.40), int(this->WIDTH * 0.13), int(this->HEIGHT * 0.05));

    // 第一列：标签
    this->labelPortName = new QLabel("串口号",this); // 1-1
    this->labelPortName->setGeometry(int(this->WIDTH * 0.05), int(this->HEIGHT * 0.5), int(this->WIDTH * 0.10), int(this->HEIGHT * 0.05));

    this->labelBaudrate = new QLabel("波特率",this); // 1-2
    this->labelBaudrate->setGeometry(int(this->WIDTH * 0.05), int(this->HEIGHT * 0.6), int(this->WIDTH * 0.10), int(this->HEIGHT * 0.05));

    this->labelDatabits = new QLabel("数据位",this); // 1-3
    this->labelDatabits->setGeometry(int(this->WIDTH * 0.05), int(this->HEIGHT * 0.7), int(this->WIDTH * 0.10), int(this->HEIGHT * 0.05));

    this->labelStopbits = new QLabel("停止位",this); // 1-4
    this->labelStopbits->setGeometry(int(this->WIDTH * 0.05), int(this->HEIGHT * 0.8), int(this->WIDTH * 0.10), int(this->HEIGHT * 0.05));

    this->labelParitybits = new QLabel("校验位",this); // 1-5
    this->labelParitybits->setGeometry(int(this->WIDTH * 0.05), int(this->HEIGHT * 0.9), int(this->WIDTH * 0.10), int(this->HEIGHT * 0.05));

    // 第二列：复选框
    this->boxPortName = new QComboBox(this); // 2-1
    this->boxPortName->setGeometry(int(this->WIDTH * 0.15), int(this->HEIGHT * 0.5), int(this->WIDTH * 0.12), int(this->HEIGHT * 0.05));

    this->boxBaudrate = new QComboBox(this); // 2-2
    this->boxBaudrate->setGeometry(int(this->WIDTH * 0.15), int(this->HEIGHT * 0.6), int(this->WIDTH * 0.12), int(this->HEIGHT * 0.05));

    this->boxDatabits = new QComboBox(this); // 2-3
    this->boxDatabits->setGeometry(int(this->WIDTH * 0.15), int(this->HEIGHT * 0.7), int(this->WIDTH * 0.12), int(this->HEIGHT * 0.05));

    this->boxStopbits = new QComboBox(this); // 2-4
    this->boxStopbits->setGeometry(int(this->WIDTH * 0.15), int(this->HEIGHT * 0.8), int(this->WIDTH * 0.12), int(this->HEIGHT * 0.05));

    this->boxParitybits = new QComboBox(this); // 2-5
    this->boxParitybits->setGeometry(int(this->WIDTH * 0.15), int(this->HEIGHT * 0.9), int(this->WIDTH * 0.12), int(this->HEIGHT * 0.05));

    // 第三列：标签、按钮
    this->labelProtocol = new QLabel("通信协议", this);
    this->labelProtocol->setGeometry(int(this->WIDTH * 0.3), int(this->HEIGHT * 0.5), int(this->WIDTH * 0.10), int(this->HEIGHT * 0.05));

    this->labelSetPawPosition = new QLabel("夹爪位置", this);
    this->labelSetPawPosition->setGeometry(int(this->WIDTH * 0.3), int(this->HEIGHT * 0.6), int(this->WIDTH * 0.10), int(this->HEIGHT * 0.05));

    this->btnReadPawPosition = new QPushButton("读取位置", this);
    this->btnReadPawPosition->setGeometry(int(this->WIDTH * 0.3), int(this->HEIGHT * 0.7), int(this->WIDTH * 0.13), int(this->HEIGHT * 0.05));

    this->btnOpenCamera = new QPushButton("打开相机", this);
    this->btnOpenCamera->setGeometry(int(this->WIDTH * 0.3), int(this->HEIGHT * 0.8), int(this->WIDTH * 0.13), int(this->HEIGHT * 0.05));

    this->btnUR5Up = new QPushButton("机械爪上升", this);
    this->btnUR5Up->setGeometry(int(this->WIDTH * 0.3), int(this->HEIGHT * 0.9), int(this->WIDTH * 0.13), int(this->HEIGHT * 0.05));

    // 横跨三四两列的编辑框
    this->editSetPawPosition = new QLineEdit(this);
    this->editSetPawPosition->setPlaceholderText("[26->80]");
    this->editSetPawPosition->setStyleSheet("QLineEdit { color : black; } QLineEdit::placeholder { color: gray; }");
    this->editSetPawPosition->setGeometry(int(this->WIDTH * 0.4), int(this->HEIGHT * 0.6), int(this->WIDTH * 0.08), int(this->HEIGHT * 0.05));

    // 第四列：复选框、按钮
    this->boxProtocol = new QComboBox(this);
    this->boxProtocol->setGeometry(int(this->WIDTH * 0.4), int(this->HEIGHT * 0.5), int(this->WIDTH * 0.15), int(this->HEIGHT * 0.05));

    this->btnSetPawPosition = new QPushButton("设置", this);
    this->btnSetPawPosition->setGeometry(int(this->WIDTH * 0.50), int(this->HEIGHT * 0.6), int(this->WIDTH * 0.08), int(this->HEIGHT * 0.05));

    this->btnReadPawIntensity = new QPushButton("读取电流", this);
    this->btnReadPawIntensity->setGeometry(int(this->WIDTH * 0.45), int(this->HEIGHT * 0.7), int(this->WIDTH * 0.13), int(this->HEIGHT * 0.05));

    this->btnCloseCamera = new QPushButton("关闭相机", this);
    this->btnCloseCamera->setGeometry(int(this->WIDTH * 0.45), int(this->HEIGHT * 0.8), int(this->WIDTH * 0.13), int(this->HEIGHT * 0.05));

    this->btnUR5Down = new QPushButton("机械爪下降", this);
    this->btnUR5Down->setGeometry(int(this->WIDTH * 0.45), int(this->HEIGHT * 0.9), int(this->WIDTH * 0.13), int(this->HEIGHT * 0.05));

    // 第五列：按钮
    this->btnPortSwitch = new QPushButton ("打开串口", this);
    this->btnPortSwitch->setGeometry(int(this->WIDTH * 0.60), int(this->HEIGHT * 0.5), int(this->WIDTH * 0.15), int(this->HEIGHT * 0.05));

    this->btnFlashPortName = new QPushButton("刷新设备", this);
    this->btnFlashPortName->setGeometry(int(this->WIDTH * 0.60), int(this->HEIGHT * 0.6), int(this->WIDTH * 0.15), int(this->HEIGHT * 0.05));

    this->btnClearEdit = new QPushButton ("清除编辑", this);
    this->btnClearEdit->setGeometry(int(this->WIDTH * 0.60), int(this->HEIGHT * 0.8), int(this->WIDTH * 0.15), int(this->HEIGHT * 0.05));

    this->btnCatchShelf = new QPushButton("抓取试管架", this);
    this->btnCatchShelf->setGeometry(int(this->WIDTH * 0.60), int(this->HEIGHT * 0.9), int(this->WIDTH * 0.15), int(this->HEIGHT * 0.05));

    // 横跨五六两列的编辑框
    this->lineEdit = new QLineEdit(this);
    this->lineEdit->setGeometry(int(this->WIDTH * 0.60), int(this->HEIGHT * 0.7), int(this->WIDTH * 0.35), int(this->HEIGHT * 0.05));

    // 第六列：按钮
    this->btnConnectUR5 = new QPushButton("连接UR5", this);
    this->btnConnectUR5->setGeometry(int(this->WIDTH * 0.80), int(this->HEIGHT * 0.5), int(this->WIDTH * 0.15), int(this->HEIGHT * 0.05));

    this->btnClearMessage = new QPushButton ("清除消息", this);
    this->btnClearMessage->setGeometry(int(this->WIDTH * 0.80), int(this->HEIGHT * 0.6), int(this->WIDTH * 0.15), int(this->HEIGHT * 0.05));

    this->btnSendMessage = new QPushButton ("发送信息", this);
    this->btnSendMessage->setGeometry(int(this->WIDTH * 0.80), int(this->HEIGHT * 0.8), int(this->WIDTH * 0.15), int(this->HEIGHT * 0.05));

    this->btnInitUR5Position = new QPushButton("回初始位置", this);
    this->btnInitUR5Position->setGeometry(int(this->WIDTH * 0.80), int(this->HEIGHT * 0.9), int(this->WIDTH * 0.15), int(this->HEIGHT * 0.05));
}

void Widget::initComboBox()
{
    // 修改复选框信息
    QStringList list1 = {"9600", "19200", "38400", "57600", "115200"};
    this->boxBaudrate->addItems(list1);
    this->boxBaudrate->setCurrentIndex(4); // 默认选择115200
    QStringList list2 = {"8位", "7位", "6位", "5位"};
    this->boxDatabits->addItems(list2);
    QStringList list3 = {"无", "1位", "2位"};
    this->boxStopbits->addItems(list3);
    this->boxStopbits->setCurrentIndex(1); // 默认选择1位
    QStringList list4 = {"无", "奇校验", "偶校验"};
    this->boxParitybits->addItems(list4);
    QStringList list5 = {"自定义协议", "ModbusRTU"};
    this->boxProtocol->addItems(list5);
    this->boxProtocol->setCurrentIndex(1); // 默认选择ModbusRTU
}

void Widget::initQTimer()
{
    this->ur5UpTimer = new QTimer(this);
    this->ur5DownTimer = new QTimer(this);
    this->ur5LeftTimer = new QTimer(this);
    this->ur5RightTimer = new QTimer(this);
    this->ur5FrontTimer = new QTimer(this);
    this->ur5BackTimer = new QTimer(this);

    this->shelfTimer = new QTimer(this);
    // 设置为单次触发
    // this->ur5UpTimer->setSingleShot(true);
    // this->ur5DownTimer->setSingleShot(true);
    // this->ur5LeftTimer->setSingleShot(true);
    // this->ur5RightTimer->setSingleShot(true);
    // this->ur5FrontTimer->setSingleShot(true);
    // this->ur5BackTimer->setSingleShot(true); 

    this->shelfTimer->setSingleShot(true);
}

void Widget::initConnection()
{
    // 绑定串口通信相关Pushbutton点击事件
    connect(this->btnPortSwitch, &QPushButton::clicked, this, &Widget::switchSerialPortStatus);
    connect(this->btnFlashPortName, &QPushButton::clicked, this, &Widget::flashSerialPortDevice);
    connect(this->btnClearMessage, &QPushButton::clicked, this, &Widget::clearMessage);
    connect(this->btnClearEdit, &QPushButton::clicked, this, &Widget::clearEdit);
    connect(this->btnSendMessage, &QPushButton::clicked, this, &Widget::sendMessage);
    
    // 绑定复选框信号
    connect(this->boxBaudrate, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &Widget::getBaudrate);
    connect(this->boxDatabits, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &Widget::getDataBits);
    connect(this->boxParitybits, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &Widget::getParity);
    connect(this->boxStopbits, static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, &Widget::getStopbits);
    // connect() // RS485切换

    // 绑定相机按钮信号
    connect(this->btnOpenCamera, &QPushButton::clicked, this->cameraWidget, &CameraWidget::openCamera);
    connect(this->btnOpenCamera, &QPushButton::clicked, this, &Widget::openCamera);
    connect(this->btnCloseCamera, &QPushButton::clicked, this->cameraWidget, &CameraWidget::closeCamera);
    connect(this->btnCloseCamera, &QPushButton::clicked, this, &Widget::closeCamera);

    // 绑定夹爪按钮信号
    connect(this->btnSetPawPosition, &QPushButton::clicked, this, &Widget::setPawPosition);
    connect(this->btnReadPawPosition, &QPushButton::clicked, this, &Widget::readPawPosition);
    connect(this->btnReadPawIntensity, &QPushButton::clicked, this, &Widget::readPawIntensity);
    connect(this->btnCatchShelf, &QPushButton::clicked, this, &Widget::catchShelf);
    connect(this->btnSetPawPosition, &QPushButton::clicked, this, &Widget::clearPawPositionEdit);

    // 连接机械臂按钮
    connect(this->btnConnectUR5, &QPushButton::clicked, this, &Widget::connectUR5);
    connect(this->btnUR5Up, &QPushButton::clicked, this, &Widget::UR5Up);
    connect(this->btnUR5Down, &QPushButton::clicked, this, &Widget::UR5Down);
    connect(this->btnInitUR5Position, &QPushButton::clicked, this, &Widget::initUR5Position);

    // 绑定接收串口数据信号
#ifndef RS485
    connect(this->serialport->getQSerialPort(), &QSerialPort::readyRead, this, &Widget::showMessage);
#else
    connect(this->rs485->getSerial(), &QSerialPort::readyRead, this, &Widget::showMessage);
#endif

    // 绑定机械臂微调按钮
    this->ur5Thread = new UR5Thread(this);
    connect(this->ur5FrontTimer, &QTimer::timeout, this, &Widget::UR5Front);
    connect(this->btnUR5Front, &QPushButton::pressed, this, [this]() {
        this->ur5FrontTimer->start(1);
    });
    connect(this->btnUR5Front, &QPushButton::released, this, [this]() {
        this->ur5FrontTimer->stop();
        // this->ur5Thread->wait();
    });

    connect(this->ur5BackTimer, &QTimer::timeout, this, &Widget::UR5Back);
    connect(this->btnUR5Back, &QPushButton::pressed, this, [this]() {
        this->ur5BackTimer->start(1);
    });
    connect(this->btnUR5Back, &QPushButton::released, this, [this]() {
        this->ur5BackTimer->stop();
        // this->ur5Thread->wait();
    });

    connect(this->ur5LeftTimer, &QTimer::timeout, this, &Widget::UR5Left);
    connect(this->btnUR5Left, &QPushButton::pressed, this, [this]() {
        this->ur5LeftTimer->start(1);
    });
    connect(this->btnUR5Left, &QPushButton::released, this, [this]() {
        this->ur5LeftTimer->stop();
        // this->ur5Thread->wait();
    });

    connect(this->ur5RightTimer, &QTimer::timeout, this, &Widget::UR5Right);
    connect(this->btnUR5Right, &QPushButton::pressed, this, [this]() {
        this->ur5RightTimer->start(1);
    });
    connect(this->btnUR5Right, &QPushButton::released, this, [this]() {
        this->ur5RightTimer->stop();
        // this->ur5Thread->wait();
    });

    connect(this->ur5UpTimer, &QTimer::timeout, this, &Widget::UR5Up);
    connect(this->btnUR5Up, &QPushButton::pressed, this, [this]() {
        this->ur5UpTimer->start(1);
    });
    connect(this->btnUR5Up, &QPushButton::released, this, [this]() {
        this->ur5UpTimer->stop();
        // this->ur5Thread->wait();
        // delete this->ur5Thread;
    });

    connect(this->ur5DownTimer, &QTimer::timeout, this, &Widget::UR5Down);
    connect(this->btnUR5Down, &QPushButton::pressed, this, [this]() {
        
        this->ur5DownTimer->start(1);
    });
    connect(this->btnUR5Down, &QPushButton::released, this, [this]() {
        this->ur5DownTimer->stop();
        // this->ur5Thread->wait();
        // delete this->ur5Thread;
    });
}

void Widget::openCamera()
{
    this->btnOpenCamera->setEnabled(false);
    this->btnCloseCamera->setEnabled(true);
}

void Widget::closeCamera()
{
    this->btnOpenCamera->setEnabled(true);
    this->btnCloseCamera->setEnabled(false);
}

void Widget::connectUR5()
{
    this->ur5->connect();
}

void Widget::setPawPosition()
{
    // 创建定时器禁用该控件一段时间
    this->btnSetPawPosition->setEnabled(false);
    QTimer * timer = new QTimer(this);
    timer->start(1000);
    disconnect(this->pawConnection);
    this->pawConnection = connect(timer, &QTimer::timeout, this->btnSetPawPosition, [this]() {
        this->btnSetPawPosition->setEnabled(true);
    });
#ifdef RS485
    QString data = this->editSetPawPosition->text();
    // 判断内容是不是浮点数，是就转换成浮点数
    bool ok;
    float f = data.toFloat(&ok);
    // qDebug() << data << "f: " << f << ok;
    if (!ok) {
        this->textBrowser->append("<span style='color: red;'>输入的不是浮点数</span>");
        return;
    }
    if (f < 26 || f > 80) {
        this->textBrowser->append("<span style='color: red;'>输入的数值不在范围内</span>");
        return;
    }
    this->rs485->pawSetClampPosition(f);
#endif
}

void Widget::clearPawPositionEdit()
{
    this->editSetPawPosition->setText("");
}

void Widget::readPawPosition()
{

}

void Widget::readPawIntensity()
{

}

void Widget::catchShelf()
{
    // 使控件失能
    this->btnCatchShelf->setEnabled(false);
    this->btnInitUR5Position->setEnabled(false);

    this->cameraWidget->openCamera();
    this->openCamera();

    auto p_Camera = this->cameraWidget->getCamera();
    cv::Mat matrix = this->cameraWidget->getMatrix();
    this->ur5->connect();
    auto p_UR5 = this->ur5;
    if (this->functionThread == nullptr) {
        // delete this->functionThread;
        this->functionThread = new QThread();
    }
    // this->functionThread = new QThread();
    // if (this->function == nullptr) {
    //     this->function = new Function();
    // }
    // 开启一个线程执行Function.cpp中的函数
    this->function->moveToThread(this->functionThread);
    disconnect(this->shelfStartConnection);
    disconnect(this->shelfFinishedConnection);
    disconnect(this->shelfTimeOutConnection);
    disconnect(this->shelfInitConnection);
    try {
        // 开启一个定时器，当定时器停止时，杀死线程
        // if (this->shelfTimer == nullptr) {
        //     this->shelfTimer = new QTimer(this);
        // }
        // 绑定信号槽
        // this->shelfStartConnection = connect(this->functionThread, &QThread::started, this, [this, p_Camera, p_UR5, matrix]() {
        //     this->function->shelf_grasp(p_Camera, p_UR5, matrix);
        // });
        // 运行时调用，保证功能在子线程中运行
        qRegisterMetaType<Camera*>("Camera*");
        qRegisterMetaType<UR5 *>("UR5*");
        qRegisterMetaType<cv::Mat>("cv::Mat");
        this->shelfStartConnection = connect(this->functionThread, &QThread::started, this, [this, p_Camera, p_UR5, matrix]() {
            QMetaObject::invokeMethod(this->function, "shelf_grasp", 
                                    Qt::QueuedConnection, 
                                    Q_ARG(Camera *, p_Camera), 
                                    Q_ARG(UR5 *, p_UR5), 
                                    Q_ARG(cv::Mat, matrix));
        });

        this->shelfFinishedConnection = connect(this->function, &Function::shelf_grasp_finished, this, [this](){ // Specify the return type as 'void'
            this->functionThread->quit();
            this->ur5->disconnect();
            this->btnInitUR5Position->setEnabled(true);
            this->btnCatchShelf->setEnabled(true);
            this->shelfTimer->stop();
            // delete this->functionThread;
        });

        this->shelfTimer->start(600000);
        this->shelfTimeOutConnection = connect(this->shelfTimer, &QTimer::timeout, this, [&]() {
            qDebug() << "Time out -> ";
            this->functionThread->quit();
            this->ur5->disconnect();
            this->btnInitUR5Position->setEnabled(true);
            this->btnCatchShelf->setEnabled(true);
            // delete this->functionThread;
        });
        this->functionThread->start();
    }
    catch (const std::exception & e) {
        qDebug() << "抓取出现异常: " << e.what();
    }
}

void Widget::initUR5Position()
{
    this->btnInitUR5Position->setEnabled(false);
    this->btnCatchShelf->setEnabled(false);

    // 初始化UR5的位置
    auto p_UR5 = this->ur5;
    this->ur5->connect();
    if (this->functionThread2 == nullptr) {
        // delete this->functionThread2;
        this->functionThread2 = new QThread();
    }
    // this->functionThread2 = new QThread();
    if (this->function2 == nullptr) {
        this->function2 = new Function();
    }
    try {
        if (this->InitArmtimer == nullptr)
            this->InitArmtimer = new QTimer(this);
        this->InitArmtimer->setSingleShot(true);

        disconnect(shelfInitConnection);
        disconnect(shelfInitFinishedConnection);
        disconnect(shelfInitTimeOutConnection);

        this->function2->moveToThread(this->functionThread2);
        qRegisterMetaType<UR5 *>("UR5*");
        this->shelfInitConnection = connect(this->functionThread2, &QThread::started, this, [this, p_UR5]() {
            QMetaObject::invokeMethod(this->function2, "init_UR5_position", 
                                    Qt::QueuedConnection, 
                                    Q_ARG(UR5 *, p_UR5));
        });

        this->shelfInitFinishedConnection = connect(this->function2, &Function::init_grasp_finished, this, [&]() {
            this->functionThread2->quit();
            this->ur5->disconnect();
            this->btnInitUR5Position->setEnabled(true);
            this->btnCatchShelf->setEnabled(true);
            this->InitArmtimer->stop();
            // delete InitArmtimer;

            // delete this->functionThread2;
        });

        this->functionThread2->start();
        InitArmtimer->start(1000);
        this->shelfInitTimeOutConnection = connect(InitArmtimer, &QTimer::timeout, this, [this]() {
            if (this->functionThread2->isRunning()) {
                return;
            }
            this->InitArmtimer->stop();
            this->functionThread2->quit();
            this->ur5->disconnect();
            this->btnInitUR5Position->setEnabled(true);
            this->btnCatchShelf->setEnabled(true);
            // delete timer;
            // disconnect(shelfInitConnection);
        });
    }
    catch (const std::exception & e) {
        qDebug() << "初始化UR5位置出现异常: " << e.what();
    }
}

// 刀具z最小：92.47 mm 0.09247 m
// 刀具z最大：450 mm 0.45 m
void Widget::UR5Up()
{
    this->ur5->connect();
    auto func = [this]() {
        auto position_tuple = this->ur5->getCurrentPositions();
        vector<double> position = std::get<1>(position_tuple);
        if (position.at(2) < 0.45) {
            position.at(2) += 0.01;
            this->ur5->moveJP(position, 1);
        }
    };
    this->ur5Thread->setFunc(func);
    this->ur5Thread->start();
    // this->ur5Thread->wait();
    this->ur5->disconnect();
}

void Widget::UR5Down()
{
    this->ur5->connect();
    auto func = [this]() {
        auto position_tuple = this->ur5->getCurrentPositions();
        vector<double> position = std::get<1>(position_tuple);
        if (position.at(2) > 0.09247) {
            position.at(2) -= 0.01;
            this->ur5->moveJP(position, 1);
        }
    };
    this->ur5Thread->setFunc(func);
    this->ur5Thread->start();
    // this->ur5Thread->wait();
    this->ur5->disconnect();
}

void Widget::UR5Left()
{
    this->ur5->connect();
    auto func = [this]() {
        auto position_tuple = this->ur5->getCurrentPositions();
        vector<double> position = std::get<1>(position_tuple);
        if (position.at(0) > -0.20079) {
            position.at(0) -= 0.01;
            this->ur5->moveJP(position, 1);
        }
    };
    this->ur5Thread->setFunc(func);
    this->ur5Thread->start();
    // this->ur5Thread->wait();
    this->ur5->disconnect();
}

void Widget::UR5Right()
{
    this->ur5->connect();
    auto func = [this]() {
        auto position_tuple = this->ur5->getCurrentPositions();
        vector<double> position = std::get<1>(position_tuple);
        if (position.at(0) < 0.13275) {
            position.at(0) += 0.01;
            this->ur5->moveJP(position, 1);
        }
    };
    this->ur5Thread->setFunc(func);
    this->ur5Thread->start();
    // this->ur5Thread->wait();
    this->ur5->disconnect();
}

void Widget::UR5Front()
{
    this->ur5->connect();
    auto func = [this]() {
        auto position_tuple = this->ur5->getCurrentPositions();
        vector<double> position = std::get<1>(position_tuple);
        if (position.at(1) < -0.38654) {
            position.at(1) += 0.01;
            this->ur5->moveJP(position, 1);
        }
    };
    this->ur5Thread->setFunc(func);
    this->ur5Thread->start();
    // this->ur5Thread->wait();
    this->ur5->disconnect();
}

void Widget::UR5Back()
{
    this->ur5->connect();
    auto func = [this]() {
        auto position_tuple = this->ur5->getCurrentPositions();
        vector<double> position = std::get<1>(position_tuple);
        if (position.at(1) > -0.68844) {
            position.at(1) -= 0.01;
            this->ur5->moveJP(position, 1);
        }
    };
    this->ur5Thread->setFunc(func);
    this->ur5Thread->start();
    // this->ur5Thread->wait();
    this->ur5->disconnect();
}
