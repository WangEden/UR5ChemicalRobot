#pragma once

#include <QThread>
#include <functional>

class UR5Thread : public QThread
{
    Q_OBJECT
public:
    explicit UR5Thread(QObject *parent = nullptr)
        : QThread(parent) {}
    explicit UR5Thread(std::function<void()> func, QObject *parent = nullptr)
        : QThread(parent), m_func(func) {}
    ~UR5Thread() {};
    void setFunc(std::function<void()> func) {
        m_func = func;
    }

protected:
    void run() override {
        m_func();
    }

private:
    std::function<void()> m_func;
};

