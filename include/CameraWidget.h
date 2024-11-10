#pragma once

#include <QWidget>
#include <QTimer>
#include <QPixmap>
#include "CameraThread.h"
#include "d435i.h"

class CameraWidget : public QWidget
{
    Q_OBJECT

public: 
    enum ImageProcessType {
        None = 0,
        SHELF = 1,
    };
    explicit CameraWidget(QWidget *parent = nullptr);
    ~CameraWidget();
    bool getCameraState() { return this->cameraState; }
    Camera * getCamera() { return this->cameraThread->getCamera(); }
    cv::Mat getMatrix() {return this->cameraThread->getMatrix();}

protected:
    void resizeEvent(QResizeEvent *event) override;
    void paintEvent(QPaintEvent *event) override;
    QPixmap mat2Pixmap(const QSize & size, const cv::Mat &mat);
    void setRunning(bool flag);

signals:
    void runningChanged(bool state);

public slots:
    void recvImage(QImage img);
    void recvCameraState(bool isOpen);
    void openCamera();
    void closeCamera();

private:
    cv::Mat mat;
    QImage image;
    CameraThread *cameraThread = nullptr;
    bool running = false;

    QPixmap cvImage;                // 相机图像
    bool cameraState = false;     // 相机是否打开
    QTimer * paintTimer;
    
    ImageProcessType imageProcessType = None;
};