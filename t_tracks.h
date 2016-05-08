#pragma once
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
class t_tracks
{
public:
	int id;
	int* bbox;
	KalmanFilter kFilt(2, 1, 0);
	int age;
	int totalVisibleCount;
	int consecutiveInvisibleCount;

	t_tracks() {};
	~t_tracks() {};
};

