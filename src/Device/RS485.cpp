#include "RS485.h"
#include <iostream>
#include <QDebug>

RS485ModbusRtuMaster::RS485ModbusRtuMaster(QObject * parent)
{
    this->timer = new QTimer();
    this->qserialport = new QSerialPort();
    connect(this->timer, &QTimer::timeout, this, &RS485ModbusRtuMaster::responseTimeoutHandle);
    connect(this->qserialport, &QSerialPort::readyRead, this, &RS485ModbusRtuMaster::serialResponseHandle);
}

RS485ModbusRtuMaster::~RS485ModbusRtuMaster()
{
    closePort();
    delete this->timer;
    delete this->qserialport;
}

QStringList RS485ModbusRtuMaster::scanPortName()
{
    QStringList m;
    foreach(const QSerialPortInfo &info,QSerialPortInfo::availablePorts())
    {
        QString s = info.portName();
        // qDebug() << s;
        m << s;
    }
    return m;
}

QSerialPort * RS485ModbusRtuMaster::getSerial()
{
    return this->qserialport;
}

// RS485ModbusRtuMaster::RS485ModbusRtuMaster(
//     const QString portName,
//     QSerialPort::BaudRate baudRate = QSerialPort::Baud115200,
//     QSerialPort::DataBits dataBits = QSerialPort::Data8,
//     QSerialPort::Parity parity = QSerialPort::NoParity,
//     QSerialPort::StopBits stopBits = QSerialPort::OneStop)
// {
//     this->portName = portName;
//     this->baudRate = baudRate;
//     this->dataBits = dataBits;
//     this->parity = parity;
//     this->stopBits = stopBits;
    
//     this->qserialport = new QSerialPort();
//     this->timer = new QTimer();
//     connect(this->timer, &QTimer::timeout, this, &RS485ModbusRtuMaster::responseTimeoutHandle);
// }

void RS485ModbusRtuMaster::openPort()
{
    this->qserialport->setPortName(this->portName);
    this->qserialport->setBaudRate(this->baudRate);
    this->qserialport->setDataBits(this->dataBits);
    this->qserialport->setParity(this->parity);
    this->qserialport->setStopBits(this->stopBits);
    this->qserialport->setFlowControl(QSerialPort::NoFlowControl);

    if (this->qserialport->open(QIODevice::ReadWrite))   
    {
        qDebug() << "串口已开启";
    } 
    else
    {
        qDebug() << "串口未开启" << qserialport->errorString();
    }
    
}

// 单独设置串口配置
void RS485ModbusRtuMaster::setPortName(const QString portName) 
{
    this->portName = portName;
}
void RS485ModbusRtuMaster::setBaudRate(QSerialPort::BaudRate baudRate)
{
    this->baudRate = baudRate;
}
void RS485ModbusRtuMaster::setDataBits(QSerialPort::DataBits dataBits)
{
    this->dataBits = dataBits;
}
void RS485ModbusRtuMaster::setParity(QSerialPort::Parity parity)
{
    this->parity = parity;
}
void RS485ModbusRtuMaster::setStopBits(QSerialPort::StopBits stopBits)
{
    this->stopBits = stopBits;
}

void RS485ModbusRtuMaster::closePort()
{
    if (this->qserialport->isOpen()) {
        this->qserialport->close();
    }
}

/**
 * @brief CRC校验值计算
 * 基于多项式为 0xA001 的 Modbus CRC16 算法
 */
uint16_t RS485ModbusRtuMaster::calculate_crc16(vector<uint8_t> & data) {
    uint16_t crc = 0xFFFF;
    for (auto byte : data) {
        crc ^= byte;
        for (int i = 0; i < 8; i++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief 数据转换
 * 单精度浮点 float 和 4字节序列 之间的精确转换（IEEE754小序规范）
 */
void RS485ModbusRtuMaster::float_2_byte(float f, vector<uint8_t> & s) {
    uint32_t longdata = 0;
    longdata = *(uint32_t*)&f;
    // longdata = *(static_cast<uint32_t*>(&f));
    s.clear();
    s.push_back((longdata & 0xFF000000) >> 24);
    s.push_back((longdata & 0x00FF0000) >> 16);
    s.push_back((longdata & 0x0000FF00) >> 8);
    s.push_back((longdata & 0x000000FF));
}
float RS485ModbusRtuMaster::byte_2_float(vector<uint8_t> s) {
    uint32_t longdata = 0;
    float f = 0.0;
    int i = 0;
    for (i = 0; i < 3; i++) {
        longdata += s[i];
        longdata <<= 8;
    }
    longdata += s[3];
    f = *(float*)&longdata;
    return f;
}

/**
 * @brief 创建一个数据帧用于发送
 * 1. 根据不同功能码创建数据帧
 * 2. CRC校验值 低字节在前，高字节在后
 * 
 */
void RS485ModbusRtuMaster::makeFrame(const ModbusRTUFrame * frame, vector<uint8_t> & byteArray) {
    // 写入设备地址
    byteArray.clear();
    byteArray.push_back(frame->address);
    
    // 写入功能代码
    ModbusFuncCode code = frame->functionCode;
    this->funcCode = code; // 发送时同时修改本类中的funcCode
    byteArray.push_back(static_cast<uint8_t>(code));
    uint16_t tmp = 0x0000;

    switch (code)
    {
    // 内容：要读取的寄存器起始地址(2B)，读取的长度(2B)
    case ReadHoldRegister: {
        tmp = (uint16_t)frame->registerStartAddress;
        byteArray.push_back((tmp & 0xFF00) >> 8);
        byteArray.push_back((tmp & 0x00FF));
        tmp = (uint16_t)frame->registerShiftAddress;
        byteArray.push_back((tmp & 0xFF00) >> 8);
        byteArray.push_back((tmp & 0x00FF));
    }break;
    
    // 内容：要写入的寄存器地址(2B)，要写入的内容(2B)
    case WriteSingleRegister: {
        tmp = (uint16_t)frame->registerStartAddress;
        byteArray.push_back((tmp & 0xFF00) >> 8);
        byteArray.push_back((tmp & 0x00FF));
        tmp = (uint16_t)frame->registerWriteContent;
        byteArray.push_back((tmp & 0xFF00) >> 8);
        byteArray.push_back((tmp & 0x00FF));
    }break;

    // 内容：要连续写入的首个寄存器地址(2B)，连续的长度(2B)，写入内容的字节数(1B)，写入的内容(4B)
    case WriteMultiRegister: {
        tmp = (uint16_t)frame->registerStartAddress;
        byteArray.push_back((tmp & 0xFF00) >> 8);
        byteArray.push_back((tmp & 0x00FF));
        tmp = (uint16_t)frame->registerShiftAddress;
        byteArray.push_back((tmp & 0xFF00) >> 8);
        byteArray.push_back((tmp & 0x00FF));
        byteArray.push_back(frame->registerMultiContentLength);
        float f_tmp = frame->registerMultiContent;
        vector<uint8_t> floatNumArray;
        float_2_byte(f_tmp, floatNumArray);
        for (uint8_t byte: floatNumArray) {
            byteArray.push_back(byte);
        }
    }break;

    default: {
        cout << "function code is invalid" << endl;
        return;
    }break;
    }

    // 添加CRC校验值
    uint16_t crc = calculate_crc16(byteArray);
    byteArray.push_back((crc & 0x00FF));
    byteArray.push_back((crc & 0xFF00) >> 8);
}

void RS485ModbusRtuMaster::sendCommand(vector<uint8_t> & data)
{
    // 检查串口是否打开
    if (!this->qserialport->isOpen()) {
        cout << "打开串口中..." << endl;;
        openPort();
    }
    // 发送时本类中存储最后一次发送的命令
    QByteArray byteArr =  vector2byteArray(data);
    this->sendData.clear();
    for (uint8_t byte: data) {
        this->sendData.append(byte);
    }
    this->qserialport->write(byteArr);
}

void RS485ModbusRtuMaster::sendCommand(QByteArray & data)
{
    // 检查串口是否打开
    if (!this->qserialport->isOpen()) {
        cout << "打开串口中..." << endl;;
        openPort();
    }
    // 发送时本类中存储最后一次发送的命令
    this->qserialport->write(data);
    this->sendData.clear();
    for (uint8_t byte: data) {
        this->sendData.append(byte);
    }
}

// 串口接收中断处理槽函数
void RS485ModbusRtuMaster::serialResponseHandle()
{
    try {
        this->response.clear();
        this->response = this->qserialport->readAll();
        QByteArray data = this->response;
        // qDebug() << "接收到的信息：" << QString::fromUtf8(this->response.toHex());
        QByteArray crcByteArray;
        size_t responseLen = data.size();
        crcByteArray.append(data.at(responseLen - 1)); // crc高位
        crcByteArray.append(data.at(responseLen - 2)); // crc低位
        data.chop(2);
        uint16_t recv_crcH = static_cast<uint16_t>(crcByteArray.at(0));
        uint16_t recv_crcL = static_cast<uint16_t>(crcByteArray.at(1));
        uint16_t recv_crc = ((recv_crcH << 8) & 0xFF00) + (recv_crcL & 0x00FF);
        vector<uint8_t> vec = byteArray2vector(data);
        if (recv_crc != calculate_crc16(vec)) // crc校验
        {
            qDebug() << "接收到的信息crc16校验失败";
        }
        this->responseDeviceId = data.at(0); // 读取响应设备的id
        uint8_t funcCode = data.at(1);
        QByteArray subarray = data.mid(2, responseLen - 4);
        switch (funcCode) // 以下根据不同的功能码和起始地址解析响应数据
        {
        case ReadHoldRegister: // 读取保持寄存器
        {
            parse03FuncData(subarray);
        }break;
        case WriteSingleRegister: // 写单个寄存器
        {

        }break;
        case WriteMultiRegister: // 写多个寄存器
        {

        }break;
        default:
            break;
        }
        this->timer->stop();
    }
    catch (exception & e) {
        qDebug() << "serialResponseHandle: " << e.what();
    }
}

// 解析 03 功能：读取夹持状态、夹持位置、夹持电流大小
void RS485ModbusRtuMaster::parse03FuncData(const QByteArray & data)
{
    uint8_t dataLen = data.at(0);
    if (dataLen == 0x02)
    {
        // 读取夹持状态时
        if (this->currentHandleStartAddr == RegisterAddrForReadClampStatus)
        {
            // 该数据高字节在前 低字节在后
            uint16_t statusH = static_cast<uint16_t>(data.at(1));
            uint16_t statusL = static_cast<uint16_t>(data.at(2));
            uint16_t status = (statusH << 8) + (statusL);
            this->pawStatus = static_cast<ClampStatus>(status);
        }
    }
    else if(dataLen == 0x04)
    {
        // 读取夹持位置
        if (this->currentHandleStartAddr == RegisterAddrForReadClampPosition)
        {
            QByteArray subarray = data.mid(1, 4);
            vector<uint8_t> position = byteArray2vector(subarray);
            this->pawPosition = byte_2_float(position);
        }
        // 读取夹持电流
        else if (this->currentHandleStartAddr == RegisterAddrForReadClampIntensity)
        {
            QByteArray subarray = data.mid(1, 4);
            vector<uint8_t> intensity = byteArray2vector(subarray);
            this->pawIntensity = byte_2_float(intensity);
        }
    }
}

vector<uint8_t> RS485ModbusRtuMaster::byteArray2vector(QByteArray & byteArray)
{
    vector<uint8_t> vec(byteArray.constData(), byteArray.constData() + byteArray.size());
    return vec;
}

QByteArray RS485ModbusRtuMaster::vector2byteArray(vector<uint8_t> & vec)
{
    QByteArray byteArray(reinterpret_cast<const char*>(vec.data()), vec.size());
    return byteArray;
}

// void RS485ModbusRtuMaster::getResponse(vector<uint8_t> & byteArray) {
//     // 检查串口是否打开
//     if (!this->qserialport->isOpen()) {
//         openPort();
//     }

//     byteArray.clear();
//     QObject::connect(&serial, &QSerialPort::readyRead, [&](){
//         QByteArray data = serial.readAll();
//         qDebug() << QString::fromUtf8(data.toHex());
//         for (uint8_t byte: data) {
//             byteArray.push_back(byte);
//         }
//     });
// }

/**
 * @brief 解析数据
 * 
 */
// void RS485ModbusRtuMaster::parseResponse(vector<uint8_t> & byteArray, vector<uint8_t> & output, uint16_t & crc) {
//     uint8_t deviceId = byteArray.at(0);
//     uint8_t funcCode = byteArray.at(1);
//     uint8_t dataLength = byteArray.at(2);
//     output.clear();
//     for (size_t i = 0; i < (size_t)dataLength; i++) {
//         output.push_back(byteArray.at(i + 3));
//     }
//     byteArray.pop_back();
//     byteArray.pop_back();
//     crc = calculate_crc16(byteArray);
// }

/**
 * @brief <报文格式>
 * 主机发送报文格式:
 * 设备地址:            01
 * 功能码：             06
 * 写入寄存器起始位置:    00 00
 * 写入寄存器内容:       00 01
 * CRC校验码：          48 0A
 * 从机（机械爪）应答报文格式:
 * 设备地址:            01
 * 功能码:              06
 * 寄存器起始地址:       00 00
 * 写入寄存器内容:       00 01
 * CRC校验码:           48 0A
 */
// 手动初始化
void RS485ModbusRtuMaster::pawManualInitialize() { 
    ModbusRTUFrame frame;
    frame.address = this->DeviceId;
    frame.functionCode = WriteSingleRegister;
    frame.registerStartAddress = (uint16_t)RegisterAddrForInit;
    this->currentHandleStartAddr = RegisterAddrForInit;
    frame.registerWriteContent = 0x0001;
    vector<uint8_t> byteArray;
    makeFrame(&frame, byteArray);
    sendCommand(byteArray);
}
// 设置设备id
void RS485ModbusRtuMaster::pawSetID(uint8_t id) { 
    ModbusRTUFrame frame;
    this->DeviceId = id;
    frame.address = this->DeviceId;
    frame.functionCode = WriteSingleRegister;
    frame.registerStartAddress = (uint16_t)RegisterAddrForSetId;
    this->currentHandleStartAddr = RegisterAddrForSetId;
    frame.registerWriteContent = 0x0001;
    vector<uint8_t> byteArray;
    makeFrame(&frame, byteArray);
    sendCommand(byteArray);
}
// 设置设备波特率
void RS485ModbusRtuMaster::pawSetBaudRate(BaudRate baudRate) { 
    ModbusRTUFrame frame;
    frame.address = this->DeviceId;
    frame.functionCode = WriteSingleRegister;
    frame.registerStartAddress = RegisterAddrForSetBaudrate;
    this->currentHandleStartAddr = RegisterAddrForSetBaudrate;
    frame.registerWriteContent = (uint16_t)baudRate;
    vector<uint8_t> byteArray;
    makeFrame(&frame, byteArray);
    sendCommand(byteArray);
}
// 设置夹持初始化方向
void RS485ModbusRtuMaster::pawSetClampInitDirection(ClampDirection clampDirection) { 
    ModbusRTUFrame frame;
    frame.address = this->DeviceId;
    frame.functionCode = WriteSingleRegister;
    frame.registerStartAddress = RegisterAddrForSetInitDiret;
    this->currentHandleStartAddr = RegisterAddrForSetInitDiret;
    frame.registerWriteContent = (uint16_t)clampDirection;
    vector<uint8_t> byteArray;
    makeFrame(&frame, byteArray);
    sendCommand(byteArray);
}
// 上电后进行自动/手动初始化的设置
void RS485ModbusRtuMaster::pawSetInitMode(bool isAuto) { 
    ModbusRTUFrame frame;
    frame.address = this->DeviceId;
    frame.functionCode = WriteSingleRegister;
    frame.registerStartAddress = RegisterAddrForSetInitAuto;
    this->currentHandleStartAddr = RegisterAddrForSetInitAuto;
    frame.registerWriteContent = (uint16_t)(isAuto ? 0x0000 : 0x0001);
    vector<uint8_t> byteArray;
    makeFrame(&frame, byteArray);
    sendCommand(byteArray);
}
// 设置是否保存参数
void RS485ModbusRtuMaster::pawSaveParameter(bool isSave) { 
    ModbusRTUFrame frame;
    frame.address = this->DeviceId;
    frame.functionCode = WriteSingleRegister;
    frame.registerStartAddress = RegisterAddrForSetIsSavaParam;
    this->currentHandleStartAddr = RegisterAddrForSetIsSavaParam;
    frame.registerWriteContent = (uint16_t)(isSave? 0x0001 : 0x0000);
    vector<uint8_t> byteArray;
    makeFrame(&frame, byteArray);
    sendCommand(byteArray);
}
// 恢复默认参数
void RS485ModbusRtuMaster::pawRecoverDefaultParam() { 
    ModbusRTUFrame frame;
    frame.address = this->DeviceId;
    frame.functionCode = WriteSingleRegister;
    frame.registerStartAddress = RegisterAddrForSetRecoverDefaultParam;
    this->currentHandleStartAddr = RegisterAddrForSetRecoverDefaultParam;
    frame.registerWriteContent = 0x0001;
    vector<uint8_t> byteArray;
    makeFrame(&frame, byteArray);
    sendCommand(byteArray);
}
// 设置IO模式打开/关闭
void RS485ModbusRtuMaster::pawSetIOMode(bool turnOn) { 
    ModbusRTUFrame frame;
    frame.address = this->DeviceId;
    frame.functionCode = WriteSingleRegister;
    frame.registerStartAddress = RegisterAddrForSetIOMode;
    this->currentHandleStartAddr = RegisterAddrForSetIOMode;
    frame.registerWriteContent = (uint16_t)(turnOn? 0x0001 : 0x0000);
    vector<uint8_t> byteArray;
    makeFrame(&frame, byteArray);
    sendCommand(byteArray);
}

/**
 * @brief <报文格式>
 * 主机发送报文格式:
 * 设备地址:            01
 * 功能码：             10
 * 写入寄存器起始位置:    00 02
 * 写入寄存器地址长度:    00 02
 * 写入数据长度:         04
 * 写入寄存器内容:       00 00 00 00
 * CRC校验码：          72 76 
 * 从机（机械爪）应答报文格式:
 * 设备地址:            01
 * 功能码:              10
 * 寄存器起始地址:       00 02
 * 修改的寄存器数量:      00 02
 * CRC校验码:           E0 08
 */
// 设置夹持位置，单位：mm(float) 设置时不断读取夹持状态
void RS485ModbusRtuMaster::pawSetClampPosition(float position) { 
    ModbusRTUFrame frame;
    frame.address = this->DeviceId;
    frame.functionCode = WriteMultiRegister;
    frame.registerStartAddress = RegisterAddrForSetClampPosition;
    this->currentHandleStartAddr = RegisterAddrForSetClampPosition;
    frame.registerShiftAddress = 0x0002;
    frame.registerMultiContentLength = 0x04;
    frame.registerMultiContent = position;
    vector<uint8_t> byteArray;
    makeFrame(&frame, byteArray);
    sendCommand(byteArray);
    // 等待当前命令的应答
    this->timer->setInterval(2000);
    while (this->timer->isActive())
    {
        /* code */
    }
    

    // --- 开启定时器监测夹持状态 ---


    // --------------------------
}
// 闭合夹爪
void RS485ModbusRtuMaster::pawSetClosePaw() { 
    pawSetClampPosition(25.f);
}
// 设置夹持速度
void RS485ModbusRtuMaster::pawSetClampSpeed(float speed) { 
    ModbusRTUFrame frame;
    frame.address = this->DeviceId;
    frame.functionCode = WriteMultiRegister;
    frame.registerStartAddress = RegisterAddrForSetClampSpeed;
    this->currentHandleStartAddr = RegisterAddrForSetClampSpeed;
    frame.registerShiftAddress = 0x0002;
    frame.registerMultiContentLength = 0x04;
    frame.registerMultiContent = speed;
    vector<uint8_t> byteArray;
    makeFrame(&frame, byteArray);
    sendCommand(byteArray);
}
// 设置夹持电流 设置时读取一次电流大小
void RS485ModbusRtuMaster::pawSetClampCurrentIntensity(float intensity) { 
    ModbusRTUFrame frame;
    frame.address = this->DeviceId;
    frame.functionCode = WriteMultiRegister;
    frame.registerStartAddress = RegisterAddrForSetClampIntensity;
    this->currentHandleStartAddr = RegisterAddrForSetClampIntensity;
    frame.registerShiftAddress = 0x0002;
    frame.registerMultiContentLength = 0x04;
    frame.registerMultiContent = intensity;
    vector<uint8_t> byteArray;
    makeFrame(&frame, byteArray);
    sendCommand(byteArray);
}

/**
 * @brief <报文格式>
 * 主机发送报文格式:
 * 设备地址:            01
 * 功能码：             03
 * 写入寄存器起始位置:    00 02
 * 写入寄存器地址长度:    00 02
 * CRC校验码：          72 76 
 * 从机（机械爪）应答报文格式:
 * 设备地址:            01
 * 功能码:              03
 * 具体情况不同
 * CRC校验码:           E0 08
 */
// 读取夹爪夹持状态 被定时器调用
RS485ModbusRtuMaster::ClampStatus RS485ModbusRtuMaster::pawReadClampStatus() { 
    ModbusRTUFrame frame;
    frame.address = this->DeviceId;
    frame.functionCode = ReadHoldRegister;
    frame.registerStartAddress = RegisterAddrForReadClampStatus;
    this->currentHandleStartAddr = RegisterAddrForReadClampStatus;
    frame.registerShiftAddress = 0x0001;
    vector<uint8_t> byteArray;
    makeFrame(&frame, byteArray);
    uint8_t crc1L = byteArray.at(byteArray.size() - 2);
    uint8_t crc1H = byteArray.at(byteArray.size() - 1);
    uint16_t crc1 = (uint16_t)(crc1L) + ((uint16_t)(crc1H) << 8);
    // sendCommand(byteArray);

    this->timer->start();
    sendCommand(byteArray);
    // 接收数据
    if (!this->timer->isActive())
    {
        return this->pawStatus;
    }
    else {
        return ClampStatus::Error;
    }
}
// 读取夹爪夹持位置
float RS485ModbusRtuMaster::pawReadClampPosition() { 
    ModbusRTUFrame frame;
    frame.address = this->DeviceId;
    frame.functionCode = ReadHoldRegister;
    frame.registerStartAddress = RegisterAddrForReadClampPosition;
    this->currentHandleStartAddr = RegisterAddrForReadClampPosition;
    frame.registerShiftAddress = 0x0002;
    vector<uint8_t> byteArray;
    makeFrame(&frame, byteArray);
    uint8_t crc1L = byteArray.at(byteArray.size() - 2);
    uint8_t crc1H = byteArray.at(byteArray.size() - 1);
    uint16_t crc1 = (uint16_t)(crc1L) + ((uint16_t)(crc1H) << 8);
    // sendCommand(byteArray);

    this->timer->start();
    sendCommand(byteArray);
    // 接收数据
    if (!this->timer->isActive())
    {
        return this->pawPosition;
    }
    else {
        return -1.;
    }
}
// 读取夹爪夹持电流大小
float RS485ModbusRtuMaster::pawReadClampCurrentIntensity() {
    ModbusRTUFrame frame;
    frame.address = this->DeviceId;
    frame.functionCode = ReadHoldRegister;
    frame.registerStartAddress = RegisterAddrForReadClampIntensity;
    this->currentHandleStartAddr = RegisterAddrForReadClampIntensity;
    frame.registerShiftAddress = 0x0002;
    vector<uint8_t> byteArray;
    makeFrame(&frame, byteArray);
    uint8_t crc1L = byteArray.at(byteArray.size() - 2);
    uint8_t crc1H = byteArray.at(byteArray.size() - 1);
    uint16_t crc1 = (uint16_t)(crc1L) + ((uint16_t)(crc1H) << 8);
    // sendCommand(byteArray);

    this->timer->start();
    sendCommand(byteArray);
    // 接收数据
    if (!this->timer->isActive())
    {
        return this->pawIntensity;
    }
    else {
        return -1.;
    }
}

/**
 * @brief 超时处理
 * 由于所有命令都是基于请求响应的，因此所有命令基于一个定时器，
 * 因此在上一次命令接收到响应或超时将不会下一次命令
 * 
 * 如果之后要处理多并发的快速操作，应修改成命令队列
 */
void RS485ModbusRtuMaster::responseTimeoutHandle()
{
    qDebug() << "接受命令超时";
}

/**
 * @brief 异常处理
 * 1.返回的异常功能码为正常功能码+0x80
 * 2.异常类型
 * 
 */

