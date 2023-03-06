#include <iostream>
#include <vector>

#include "opencv2/opencv.hpp"

#include "tracker.h"
#include "TrackingBox.h"

void setLaneNum(std::vector<TrackingBox>& trkBox, const std::vector<TrackingBox>& postTrkBox);
int getLineNum(const cv::Mat& frame, cv::Mat& lane, int x, int y);
void on_mouse(int event, int x, int y, int flags, void*);

static int frame_count = 0;
int lanePtrNum = 0;
int boxSize = 60;
std::vector<cv::Point2f> lanePtr;
std::vector<KalmanTracker> trackers;
std::vector<TrackingBox> trkBox;
std::vector<TrackingBox> postTrkBox;

void main() {
	std::string path = "./sample/resize2.avi";
	//std::string path = "D:\gpro\GOPR2879.avi";

	cv::VideoCapture capture(path);
	if (!capture.isOpened()) {
		std::cerr << "Unable to open file" << std::endl;
		return;
	}

	int firstFrame = 1;
	double fps = capture.get(cv::CAP_PROP_FPS);
	cv::Mat frame, post, out, temp;
	cv::Mat element3(3, 3, CV_8U, cv::Scalar(1));
	cv::Mat element5(5, 5, CV_8U, cv::Scalar(1));


	cv::namedWindow("frame");
	cv::namedWindow("out");
	cv::setMouseCallback("frame", on_mouse);

	capture >> post;
	capture >> frame;
	cv::imshow("frame", frame);
	cv::waitKey(0);


	cv::Mat lane(frame.rows, frame.cols, CV_8UC3);	
	for (int i = 0; i < lanePtr.size(); i += 2) {
		cv::line(lane, lanePtr[i], lanePtr[i + 1], cv::Scalar(0, 0, 255), 2);
	}

	cv::Mat laneMask;
	cv::cvtColor(lane, laneMask, cv::COLOR_BGR2GRAY);
	
	//cv::waitKey(1000 / fps) < 1
	//cv::waitKey(1) < 1
	while (cv::waitKey(1) < 1) {
		cv::absdiff(frame, post, out);

		cv::medianBlur(out, out, 3);
		/*
		cv::erode(out, out, element3, cv::Point(-1, -1), 1);
		*/
		cv::dilate(out, out, element3, cv::Point(-1, -1), 1);
		
		cv::cvtColor(out, out, cv::COLOR_BGR2GRAY);
		cv::threshold(out, out, 10, 255, cv::THRESH_BINARY);


		cv::Mat labels, stats, centroid;
		std::vector<TrackingBox> rectData;
		int labelNum = cv::connectedComponentsWithStats(out, labels, stats, centroid);

		cv::cvtColor(out, out, cv::COLOR_GRAY2BGR);	// 임시

		out.setTo(cv::Scalar::all(255), lane);

		for (int i = 1; i < labelNum; i++) {
			double* center = centroid.ptr<double>(i);
			int* stat = stats.ptr<int>(i);

			if (stat[4] > 300 && stat[0] < 500) {
				//std::cout << "X : " << center[0] << ", Y : " << center[1] << std::endl;
				cv::Rect rect(stat[0], stat[1], stat[2], stat[3]);

				//cv::putText(out, std::to_string(stat[4]), cv::Point(center[0], center[1]), 1, 1, cv::Scalar(0, 255, 0));
				//cv::circle(out, cv::Point(center[0], center[1]), 5, cv::Scalar(255, 0, 0), -1);
				//cv::rectangle(out, rect, cv::Scalar(255, 0, 0));

				getLineNum(out, laneMask, center[0], center[1]);

				
				TrackingBox tempBox;
				cv::Rect centroid_Rect(cv::Point(center[0] - boxSize, center[1] - boxSize), 
					cv::Point(center[0] + boxSize, center[1] + boxSize));
				tempBox.box = centroid_Rect;
				tempBox.id = -1, tempBox.frame = frame_count;
				rectData.push_back(tempBox);
				
			}
		}

		{
			trkBox = TestSORT(rectData, trackers);
			setLaneNum(trkBox, postTrkBox);
;
			// 차선 변경 감지
			for (auto& i : trkBox) {
				int pX = i.box.x + boxSize;
				int pY = i.box.y + boxSize;
				int num = getLineNum(out, laneMask, pX, pY);

				//std::cout << i.lane << ", " << num << std::endl;

				cv::putText(out, std::to_string(i.id), cv::Point(i.box.x, i.box.y), 1, 2, cv::Scalar(0, 255, 0), 3);
				// 차선이 바뀌었다면
				if (i.lane != -1 && num != i.lane) {
					std::cout << "Detected!" << std::endl;
					cv::circle(frame, cv::Point(pX, pY), 25, cv::Scalar(0, 255, 255), -1);
					cv::imshow("frame", frame);
					//cv::waitKey(0);
				}
				//std::cout << "Num : " << num << std::endl;
				//std::cout << "Lane num : " << i.lane << std::endl;
				i.lane = num;
			}
			postTrkBox = trkBox;
		}
		

		cv::imshow("frame", frame);
		cv::imshow("out", out);
		//cv::imshow("lane", lane);

		capture >> post;
		capture >> frame;
	}


}

cv::Mat&& getLaneMatrix(const cv::Mat& src) {

}

void on_mouse(int event, int x, int y, int flags, void*) {
	if (event == cv::EVENT_LBUTTONDOWN) {
		lanePtr.push_back(cv::Point2f(x, y));
		lanePtrNum++;
	}
}

//x 좌표를 받아서 x 좌표까지 라인이 몇개있는지 // 값이 몇번 바뀌는지
int getLineNum(const cv::Mat& frame, cv::Mat& mask, int x, int y) {
	int lineNum = 0;
	int currentValue;

	const uchar* p = mask.ptr<uchar>(y);
	currentValue = p[0];
	for (int i = 0; i < x; i++) {
		//std::cout << currentValue << " ";
		if (p[i] != currentValue) {
			currentValue = p[i];
			lineNum++;
		}
		//cv::circle(frame, cv::Point(i, y), 1, cv::Scalar(255, 255, 0), 1);
	}

	cv::line(frame, cv::Point(0, y), cv::Point(x,y), cv::Scalar(255, 255, 0), 1);
	lineNum = lineNum / 2;
	//cv::putText(frame, "Line : " + std::to_string(lineNum), cv::Point(x+50, y+50), 1, 2, cv::Scalar(0, 255, 0), 3);
	cv::circle(frame, cv::Point(x, y), 10, cv::Scalar(255, 0, 0), 1);

	return lineNum;
}

void setLaneNum(std::vector<TrackingBox>& trkBox, const std::vector<TrackingBox>& postTrkBox) {
	for (auto& i : trkBox) {
		for (auto& j : postTrkBox) {
			if (i.id == j.id) {
				i.lane = j.lane;
				continue;
			}
		}
	}
}