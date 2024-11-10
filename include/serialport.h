#ifndef SERIAL_PORT
#define SERIAL_PORT

#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>

class SerialPort
{
public:
    SerialPort();
    ~SerialPort();
    void serialPortInit(
        QString serialportname,
        QSerialPort::BaudRate baudrate,
        QSerialPort::DataBits dataBits,
        QSerialPort::Parity parity,
        QSerialPort::StopBits stopbits);
    QStringList scanPortName(); // 扫描串口设备
    void openPort();
    void closePort();
    void sendMessage(QString & data);
    void sendMessage(QByteArray & data);
    QSerialPort * getQSerialPort();
    

private:
    QSerialPort * qserialport = nullptr;
    QString serialportname;
    QSerialPort::BaudRate baudRate;
    QSerialPort::DataBits dataBits;
    QSerialPort::Parity parity;
    QSerialPort::StopBits stopbits;
};
#endif