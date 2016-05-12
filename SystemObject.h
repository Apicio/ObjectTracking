#pragma once

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/opencv.hpp"
#include "Intestazione.h"
#include <iostream>
#include <stdio.h>
#include "t_tracks.h"
#include "Intestazione.h"
#include <string>
#include <vector>
#include <Math.h>

#define THRESHOLD 0.7
#define INVISIBLEFORTOOLONG 20
#define AGETHRESHOLD 8
#define VISIBILITYTHRESHOLD 0.6
#define DMAX 100
#define MINAREA 200
#define MAXAREA 2500

using namespace cv;
using namespace std;

class SystemObject
{
private:
	Ptr<BackgroundSubtractor> pMOG;
public:
	SystemObject() {
		pMOG = createBackgroundSubtractorKNN(500, 100, true); /*history, ngaussianMixuter, shadows*/
	};
	void detectObjects(const Mat, vector<double*>&, vector<double*>&, Mat&);
	void predictNewLocationsOfTracks(vector<t_tracks>&);
	void detectionToTrackAssignment(vector<t_tracks>, vector<double*>, vector<int*>&, vector<int>&, vector<int>&);
	void updateAssignedTracks(vector<double*>, vector<double*>, vector<int*>, vector<t_tracks>&);
	void updateUnassignedTracks(vector<int>&, vector<t_tracks>&);
	void deleteLostTracks(vector<t_tracks>&);
	void createNewTracks(vector<double*>, vector<double*>, vector<int>, int&, vector<t_tracks>&);
	~SystemObject();
};
