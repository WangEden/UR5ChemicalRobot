#include "CameraThread.h"
#include "d435i.h"

CameraThread::CameraThread(QObject *parent) : QThread(parent)
{
    this->camera = new Camera();
}

CameraThread::~CameraThread()
{
    if (cameraIsOpened)
    {
        closeCamera();
    }
}

void CameraThread::openCamera()
{
    if (!cameraIsOpened)
    {
        auto matrix_tuple = this->camera->connect();
        this->matrix = get<0>(matrix_tuple);
        cameraIsOpened = true;
        emit cameraIsOpen(true);
    }
}

void CameraThread::closeCamera()
{
    if (cameraIsOpened)
    {
        cameraIsOpened = false;
        camera->disconnect();
        delete camera;
        emit cameraIsOpen(false);
    }
}

cv::Mat CameraThread::getMat() const
{
    cv::Mat mat;
    matLock.lock();
    mat_.copyTo(mat);
    matLock.unlock();
    return mat;
}

void CameraThread::setMat(const cv::Mat &mat)
{
    matLock.lock();
    mat.copyTo(mat_);
    matLock.unlock();
}

void CameraThread::run()
{
    openCamera();
    while (!isInterruptionRequested())
    {
        tuple<cv::Mat, cv::Mat> images = camera->get_images();
        this->mat_ = get<0>(images);
        this->setMat(this->mat_);
        emit sendMat(this->mat_);
        emit sendImage(toQImage(this->mat_));
        msleep(30);
    }
}

QImage CameraThread::toQImage(cv::Mat &srcFrom)
{
    cv::Mat rgbFrame;
    cv::cvtColor(srcFrom, rgbFrame, cv::COLOR_BGR2RGB);
    QImage img((const uchar *)rgbFrame.data, rgbFrame.cols, rgbFrame.rows, rgbFrame.step, QImage::Format_RGB888);
    img.bits();
    return img;
}
