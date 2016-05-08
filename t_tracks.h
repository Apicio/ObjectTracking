#pragma once
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"

using namespace cv;

class t_tracks
{
public:
	int id;
	int* bbox;
	KalmanFilter kalmanFilter;
	int age;
	int totalVisibleCount;
	int consecutiveInvisibleCount;

	t_tracks() {};
	~t_tracks() {};
};

