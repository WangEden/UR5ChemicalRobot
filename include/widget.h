#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QComboBox>
#include <QTextBrowser>
#include <QLineEdit>
#include "CameraWidget.h"
#include "UR5.h"
#include "serialport.h"
#include <string>
#include "RS485.h"
#include "Function.h"
#include "UR5Thread.h"


class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();

private slots:
    void switchSerialPortStatus();
    void flashSerialPortDevice();
    void clearMessage();
    void sendMessage();
    void getBaudrate(int index);
    void getDataBits(int index);
    void getParity(int index);
    void getStopbits(int index);
    void clearEdit();
    void showMessage();
    void openCamera();
    void closeCamera();
    void setPawPosition();
    void clearPawPositionEdit();
    void readPawPosition();
    void readPawIntensity();
    void catchShelf();
    void connectUR5();
    void initUR5Position();
    void UR5Up();
    void UR5Down();
    void UR5Front();
    void UR5Back();
    void UR5Left();
    void UR5Right();
    

private:
    int WIDTH = 1800;
    int HEIGHT = 1200;
    // UI 组件
    QPushButton * btnPortSwitch = nullptr;
    QPushButton * btnClearMessage = nullptr;
    QPushButton * btnFlashPortName = nullptr;
    QPushButton * btnClearEdit = nullptr;
    QPushButton * btnSendMessage = nullptr;
    QLabel * labelPortName = nullptr;
    QLabel * labelBaudrate = nullptr;
    QLabel * labelDatabits = nullptr;
    QLabel * labelStopbits = nullptr;
    QLabel * labelParitybits = nullptr;
    QComboBox * boxPortName = nullptr;
    QComboBox * boxBaudrate = nullptr;
    QComboBox * boxDatabits = nullptr;
    QComboBox * boxStopbits = nullptr;
    QComboBox * boxParitybits = nullptr;
    QTextBrowser * textBrowser = nullptr;
    QLineEdit * lineEdit = nullptr;
    QLabel * labelProtocol = nullptr;
    QComboBox * boxProtocol = nullptr;

    CameraWidget * cameraWidget = nullptr;    
    QPushButton * btnOpenCamera = nullptr;
    QPushButton * btnCloseCamera = nullptr;

    QLabel * labelSetPawPosition = nullptr;
    QLineEdit * editSetPawPosition = nullptr;
    QPushButton * btnSetPawPosition = nullptr;

    QPushButton * btnReadPawPosition = nullptr;
    QPushButton * btnReadPawIntensity = nullptr;
    QPushButton * btnCatchShelf = nullptr;

    // 串口设备
    SerialPort * serialport = nullptr;
    QSerialPort::BaudRate baudrate;
    QSerialPort::DataBits dataBits;
    QSerialPort::Parity parity;
    QSerialPort::StopBits stopbits;

    RS485ModbusRtuMaster * rs485 = nullptr;

    QPushButton * btnConnectUR5 = nullptr;
    QPushButton * btnDisconnectUR5 = nullptr;
    QPushButton * btnUR5Up = nullptr;
    QPushButton * btnUR5Down = nullptr;
    QPushButton * btnUR5Front = nullptr;
    QPushButton * btnUR5Back = nullptr;
    QPushButton * btnUR5Left = nullptr;
    QPushButton * btnUR5Right = nullptr;
    UR5Thread * ur5Thread = nullptr;

    QPushButton * btnInitUR5Position = nullptr;

    class Utils {
        void splitString(const std::string & in_, std::vector<std::string> & out, char split_);
    };

    void drawWidget();
    void initComboBox();
    void initQTimer();
    void initConnection();

    QMetaObject::Connection pawConnection;
    QMetaObject::Connection shelfStartConnection;
    QMetaObject::Connection shelfFinishedConnection;
    QMetaObject::Connection shelfTimeOutConnection;
    QMetaObject::Connection shelfInitConnection;
    QMetaObject::Connection shelfInitFinishedConnection;
    QMetaObject::Connection shelfInitTimeOutConnection;

    QThread * functionThread = nullptr;
    QThread * functionThread2 = nullptr;
    Function * function = nullptr;
    Function * function2 = nullptr;
    QTimer * shelfTimer = nullptr;
    QTimer * InitArmtimer = nullptr;

    UR5 * ur5 = nullptr;

    QTimer * ur5UpTimer = nullptr;
    QTimer * ur5DownTimer = nullptr;

    QTimer * ur5FrontTimer = nullptr;
    QTimer * ur5BackTimer = nullptr;
    QTimer * ur5LeftTimer = nullptr;
    QTimer * ur5RightTimer = nullptr;
};
#endif // WIDGET_H
