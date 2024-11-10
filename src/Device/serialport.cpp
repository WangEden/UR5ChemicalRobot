# include "serialport.h"
#include <QDebug>

SerialPort::SerialPort()
{
    this->qserialport = new QSerialPort();
}


SerialPort::~SerialPort()
{
    delete [] this->qserialport;
}

void SerialPort::serialPortInit(
    QString serialportname,
    QSerialPort::BaudRate baudrate,
    QSerialPort::DataBits dataBits,
    QSerialPort::Parity parity,
    QSerialPort::StopBits stopbits)
{
    this->serialportname = serialportname;
    this->baudRate = baudrate;
    this->dataBits = dataBits;
    this->parity = parity;
    this->stopbits = stopbits;
}

QStringList SerialPort::scanPortName()
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

void SerialPort::openPort()
{
    this->qserialport->setPortName(this->serialportname);
    this->qserialport->setBaudRate(baudRate);
    this->qserialport->setDataBits(this->dataBits);
    this->qserialport->setParity(this->parity);
    this->qserialport->setStopBits(this->stopbits);
    this->qserialport->setFlowControl(QSerialPort::NoFlowControl);
    if (this->qserialport->open(QIODevice::ReadWrite))
    {
        qDebug() << "串口已开启";
        // // 发送Hello
        // QString data = "Hello";
        // QByteArray sendData;
        // sendData = data.toUtf8(); // 转UTF8格式的字节流发送
        // // sendData = QByteArray::fromHex (data.toLatin1().data());//按十六进制编码发送
        // qserialport->write(sendData);
    }
    else
    {
        qDebug() << "串口未打开" << qserialport->errorString();
    }
}

void SerialPort::closePort()
{
    this->qserialport->close();
    qDebug() << "串口已关闭";
}

void SerialPort::sendMessage(QString & data)
{
    // 发送Hello
    QByteArray sendData;
    sendData = data.toUtf8(); // 转UTF8格式的字节流发送
    // sendData = QByteArray::fromHex (data.toLatin1().data());//按十六进制编码发送
    qserialport->write(sendData);
}

void SerialPort::sendMessage(QByteArray & data)
{
    qserialport->write(data);
}

QSerialPort * SerialPort::getQSerialPort()
{
    return this->qserialport;
}

