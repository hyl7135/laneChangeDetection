#pragma once

#include "opencv2/opencv.hpp"

typedef struct TrackingBox
{
	int frame;
	int id;
	cv::Rect_<float> box;

	int lane = -1;
}TrackingBox;
