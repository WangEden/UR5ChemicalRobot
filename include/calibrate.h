#ifndef CALIBRATE_H
#define CALIBRATE_H

#include "d435i.h"
#include "UR5.h"

using namespace std;

tuple<cv::Mat, cv::Mat> calibrate();

#endif