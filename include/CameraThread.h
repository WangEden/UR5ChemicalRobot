#pragma once
#include "d435i.h"

#include <QThread>
#include <QTimer>
#include <QImage>
#include <QMutex>

class CameraThread : public QThread
{
    Q_OBJECT
public:
    explicit CameraThread(QObject *parent = nullptr);
    ~CameraThread();

    void openCamera();
    void closeCamera();
    cv::Mat getMat() const;
    void setMat(const cv::Mat &mat);
    void recvStatus(int status);
    Camera *getCamera() { return camera; }
    cv::Mat getMatrix() { return this->matrix;}

protected:
    void run() override;

signals:
    void cameraIsOpen(bool);            // 相机打开信号
    void sendImage(QImage);             // 发送图像信号
    void sendMat(cv::Mat);

private:
    Camera *camera; // 相机对象
    QTimer *cameraTimer;
    bool cameraIsOpened = false; // 相机是否打开
    cv::Mat mat_;
    mutable QMutex matLock;
    QImage toQImage(cv::Mat &srcFrom);
    cv::Mat matrix;
};