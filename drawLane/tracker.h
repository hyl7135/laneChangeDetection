#pragma once

#include "opencv2/opencv.hpp"

#include "TrackingBox.h"
#include "KalmanTracker.h"

using namespace std;
using namespace cv;

// Computes IOU between two bounding boxes
double GetIOU(Rect_<float> bb_test, Rect_<float> bb_gt);
std::vector<TrackingBox> TestSORT(std::vector<TrackingBox>& detFrameData, std::vector<KalmanTracker>& trackers);