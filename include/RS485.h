#pragma once

#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <QDebug>
#include <QTimer>

#include <qobjectdefs.h>
#include <vector>
#include <cstdint> // for uint8_t and uint16_t

using namespace std;

class RS485ModbusRtuMaster : public QObject
{
    Q_OBJECT

public:
    // 功能码
    enum ModbusFuncCode {
        ReadHoldRegister    = 0x03, // 读取保持寄存器
        WriteSingleRegister = 0x06, // 写单个寄存器
        WriteMultiRegister  = 0x10, // 写多个寄存器
    };

    // 数据帧
    struct ModbusRTUFrame {
        uint8_t address = 0x01;             // 从站地址
        ModbusFuncCode functionCode;        // 功能码
        uint16_t registerStartAddress = 0x0000; // 寄存器起始地址
        uint16_t registerShiftAddress = 0x0000; // 从起始地址起的长度
        uint16_t registerWriteContent = 0x0000; // 要写入的内容，写入单一寄存器时使用
        uint8_t registerMultiContentLength = 0x00; // 写入多个寄存器时数据长度
        float registerMultiContent = 0.0f; // 写入多个寄存器时的数据
        uint16_t crc = 0x0000;                // CRC校验值
    };

    // 夹爪波特率参数
    enum BaudRate {
        baud9600 = 0x0000,
        baud19200 = 0x0001,
        baud38400 = 0x0002,
        baud57600 = 0x0003,
        baud115200 = 0x0004,
        baud153600 = 0x0005,
        baud256000 = 0x0006,
    };

    // 夹爪夹持状态
    enum ClampStatus {
        Arrived = 0x0000,
        Proceeding = 0x0001,
        Clamped = 0x0002,
        Drop = 0x0003,
        Error = 0x0004,
    };

    enum ClampDirection {
        ClampOpen = 0x0000,
        ClampClose = 0x0001,
    };

    // 一些功能的寄存器起始地址
    enum RegisterStartAddr {
        RegisterAddrForInit = 0x0000,
        RegisterAddrForSetClampPosition = 0x0002,
        RegisterAddrForSetClampSpeed = 0x0004,
        RegisterAddrForSetClampIntensity = 0x0006,

        RegisterAddrForReadClampStatus = 0x0041,
        RegisterAddrForReadClampPosition = 0x0042,
        // RegisterAddrForReadClampSpeed = 0x0044,
        RegisterAddrForReadClampIntensity = 0x0046,

        RegisterAddrForSetId = 0x0080,
        RegisterAddrForSetBaudrate = 0x0081,
        RegisterAddrForSetInitDiret = 0x0082,
        RegisterAddrForSetInitAuto = 0x0083,
        RegisterAddrForSetIsSavaParam = 0x0084,
        RegisterAddrForSetRecoverDefaultParam = 0x0085,

        RegisterAddrForSetIOMode = 0x0090,
    };

    RS485ModbusRtuMaster(QObject * parent = nullptr);
    // RS485ModbusRtuMaster(const QString portName,
    //     QSerialPort::BaudRate baudRate,
    //     QSerialPort::DataBits dataBits,
    //     QSerialPort::Parity parity,
    //     QSerialPort::StopBits stopBits);
    ~RS485ModbusRtuMaster();
    QSerialPort * getSerial();
    QStringList scanPortName();
    void openPort();
    void closePort();

    void setPortName(const QString portName);
    void setBaudRate(QSerialPort::BaudRate baudRate);
    void setDataBits(QSerialPort::DataBits );
    void setParity(QSerialPort::Parity);
    void setStopBits(QSerialPort::StopBits);

    // 功能函数 06 写入单个寄存器
    void pawManualInitialize(); // 手动初始化
    void pawSetID(uint8_t id); // 设置设备id
    void pawSetBaudRate(BaudRate); // 设置设备波特率
    void pawSetClampInitDirection(ClampDirection); // 设置夹持初始化方向
    void pawSetInitMode(bool isAuto); // 上电后进行自动/手动初始化的设置
    void pawSaveParameter(bool isSave); // 保存参数
    void pawRecoverDefaultParam(); // 恢复默认参数
    void pawSetIOMode(bool turnOn); // 设置IO模式打开/关闭

    // 功能函数 10 写多个寄存器
    void pawSetClampPosition(float); // 设置夹持位置，单位：mm
    void pawSetClosePaw(); // 闭合夹爪
    void pawSetClampSpeed(float); // 设置夹持速度
    void pawSetClampCurrentIntensity(float); // 设置夹持电流

    // 功能函数 03 读
    ClampStatus pawReadClampStatus(); // 读取夹爪夹持状态
    float pawReadClampPosition(); // 读取夹爪夹持位置
    float pawReadClampCurrentIntensity(); // 读取夹爪夹持电流大小

    // CRC校验值计算
    uint16_t calculate_crc16(vector<uint8_t> & data);

    // 单精度浮点 float 和 4字节序列 之间的精确转换
    void float_2_byte(float f, vector<uint8_t> & s);
    float byte_2_float(vector<uint8_t> s);

    // 创建数据报文
    void makeFrame(const ModbusRTUFrame *, vector<uint8_t> &);
    void sendCommand(vector<uint8_t> & data);
    void sendCommand(QByteArray & data);

    vector<uint8_t> byteArray2vector(QByteArray & byteArray);
    QByteArray vector2byteArray(vector<uint8_t> & vec);

    QByteArray getTheResponseVariable() {return this->response;};

private slots:
    // 解析数据报文
    void serialResponseHandle();
    // void getResponse(vector<uint8_t> & byteArray);
    // void parseResponse(vector<uint8_t> & byteArray, vector<uint8_t> & output, uint16_t & crc);
    void responseTimeoutHandle();

private:
    uint8_t DeviceId = 0x01;
    ModbusFuncCode funcCode = ReadHoldRegister;
    QByteArray sendData;
    QByteArray response;
    uint8_t responseDeviceId = 0x01; // 默认01
    ClampStatus pawStatus; // 存储爪子夹持状态的响应
    float pawPosition; // 存储爪子夹持位置的响应
    float pawIntensity; // 存储爪子夹持电流的响应
    RegisterStartAddr currentHandleStartAddr; // 存储当前命令操作的寄存器起始地址
    QTimer * timer;

    QSerialPort * qserialport;
    QString portName;
    QSerialPort::BaudRate baudRate = QSerialPort::Baud115200;
    QSerialPort::DataBits dataBits = QSerialPort::Data8;
    QSerialPort::Parity parity = QSerialPort::NoParity;
    QSerialPort::StopBits stopBits = QSerialPort::OneStop;

    

    void parse03FuncData(const QByteArray & data);
};
