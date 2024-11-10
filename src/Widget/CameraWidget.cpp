#include "CameraWidget.h"
#include <QDebug>
#include <QPainter>


CameraWidget::CameraWidget(QWidget *parent) : QWidget(parent)
{
    // this->paintTimer = new QTimer();
    // this->paintTimer->setInterval(100);
}

CameraWidget::~CameraWidget()
{
    // delete this->paintTimer;
    if (this->cameraThread != nullptr)
    {
        this->cameraThread->requestInterruption();
        this->cameraThread->wait();
        delete this->cameraThread;
    }
}

void CameraWidget::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);
}

void CameraWidget::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);

    // 设置背景颜色
    painter.setPen(QColor(192, 192, 192));
    painter.setBrush(QBrush(QColor(192, 192, 192)));
    painter.drawRect(0, 0, this->width(), this->height());
    // 等比缩放，绘制图像
    if (this->cameraState)
    {
        if (!this->cvImage.isNull())
        {
            // 寻找图像大小和widget大小的缩放关系
            QRectF imgRect = this->cvImage.rect();
            QRectF widgetRect = this->rect();
            double factor = qMin(widgetRect.width() / imgRect.width(), widgetRect.height() / imgRect.height());
            // 计算新的Rect
            int imgWidth = imgRect.width() * factor;
            int imgHeight = imgRect.height() * factor;
            int startX = (this->width() - imgWidth) / 2;
            int startY = (this->height() - imgHeight) / 2;
            // 显示图像
            painter.drawPixmap(startX, startY, imgWidth, imgHeight, this->cvImage);
        }
    }

    QWidget::paintEvent(event);
}

void CameraWidget::openCamera()
{
    if (this->cameraThread == nullptr) {
        this->cameraThread = new CameraThread();
        connect(this->cameraThread, &CameraThread::sendImage, this, &CameraWidget::recvImage);
        connect(this->cameraThread, &CameraThread::cameraIsOpen, this, &CameraWidget::recvCameraState);
        this->cameraThread->start();
    }
}

void CameraWidget::closeCamera()
{
    if (this->cameraThread != nullptr)
    {
        // 关闭相机，等待线程结束
        this->cameraThread->requestInterruption();
        this->cameraThread->wait();
        delete this->cameraThread;
        this->cameraThread = nullptr;
    }
    update();
}

void CameraWidget::recvImage(QImage img)
{
    this->image = img;
    // 转换为QPixmap
    this->cvImage = QPixmap::fromImage(this->image);
    update();
}

void CameraWidget::recvCameraState(bool isOpen)
{
    this->cameraState = isOpen;
    emit runningChanged(isOpen);
}

