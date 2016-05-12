#pragma once

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/opencv.hpp"
#include "Intestazione.h"
#include "SystemObject.h"
#include "t_tracks.h"
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <sstream>
#include "t_Mat.h"
#include <string>
#include <math.h>
#include <vector>


using namespace cv;
using namespace std;

void displayTrackingResults(Mat, Mat, vector<double*>, vector<double*>, vector<t_tracks>);

int main(int argc, char** argv)
{
	vector<t_tracks> tracks;
	int nextId = 1;
	SystemObject* obj = new SystemObject();
	namedWindow("Display Original", WINDOW_AUTOSIZE); 
	namedWindow("Display Mask", WINDOW_AUTOSIZE);

	for (int i = 1; i < 795; i++) {
		/** Declaration ************************************/
		vector<double*> bboxes;
		vector<int*> assignments;
		vector<int> unassignedTracks;
		vector<int> unassignedDetections;
		vector<double*> centroids;
		stringstream toOpen;
		Mat mask, frame;
		/** Declaration ************************************/
		toOpen << "view1/" << i << ".jpg";
		frame = imread(toOpen.str(), IMREAD_COLOR);

		obj->detectObjects(frame, /*return*/ centroids, bboxes, mask);
	//	obj->predictNewLocationsOfTracks(tracks);
		obj->detectionToTrackAssignment(tracks, centroids, /*return*/ assignments, unassignedTracks, unassignedDetections);
		obj->updateAssignedTracks(centroids, bboxes, assignments, /*return*/ tracks);
		obj->updateUnassignedTracks(unassignedTracks,  /*return*/ tracks);
		obj->deleteLostTracks(/*return*/ tracks);
		obj->createNewTracks(centroids, bboxes, unassignedDetections, /*return*/ nextId, tracks);
		displayTrackingResults(frame, mask, centroids,bboxes, tracks);

		/** Destroy *****************************************/
		while (bboxes.size() != 0) {
			double* arr = bboxes.back();
			delete[]arr;
			bboxes.pop_back();
		}
		while (assignments.size() != 0) {
			int* arr = assignments.back();
			delete[]arr;
			assignments.pop_back();
		}
		while (unassignedTracks.size() != 0) {
			unassignedTracks.pop_back();
		}
		while (unassignedDetections.size() != 0) {
			unassignedDetections.pop_back();
		}
		while (centroids.size() != 0) {
			double* arr = centroids.back();
			delete[]arr;
			centroids.pop_back();
		}
		/** Destroy *****************************************/
	}


	return 0;
}
/*To check drawing rectangles*/
void displayTrackingResults(Mat frame, Mat mask, vector<double*> centroids, vector<double*> bboxes, vector<t_tracks> tracks) {
#if 0
	int numTraks = bboxes.size();
	for (int i = 0; i < numTraks; i++) {
	int cur_box[4] = { bboxes.at(i)[0], bboxes.at(i)[1],bboxes.at(i)[2],bboxes.at(i)[3] };
	Point pt1(cur_box[0], cur_box[1]);
	Point pt2(cur_box[0]+cur_box[2], cur_box[1]+cur_box[3]);
	rectangle(frame, pt1, pt2, 0);
	//putText(img, label, Point(x, y), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2.0);
	}

	imshow("Display Original", frame);
	imshow("Display Mask", mask);

	waitKey(0); // Wait for a keystroke in the window
#endif
#if 1
	int numTraks = tracks.size();
	for (int i = 0; i < numTraks; i++) {
		double* cur_box = tracks.at(i).bbox;
		Point pt1(cur_box[0], cur_box[1]);
		Point pt2(cur_box[0]+cur_box[2], cur_box[1]+cur_box[3]);
		rectangle(frame, pt1, pt2, 0);
		stringstream label; 
		label << "track: " <<  tracks.at(i).id << "";
		putText(frame, label.str(), pt1, FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2.0);
	}

	imshow("Display Original", frame);
	imshow("Display Mask", mask);

	waitKey(0); // Wait for a keystroke in the window
#endif
}


// [VISIONE ARTIFICIALE]-Tracking
/**
T1 = threshold_value we use only one value because of semplicity
Create Similarity Matrix (Distance){
take objects;
take blobs;

FOR EACH blob,object in blobs,objects
S(blob,object) = computeSimilarity(blob,object) = 1-dbo/dmax -> We compute the center of box, the center of object and after we calculate the difference
return S;
}

FOR EACH frame{
S = Create Similarity Matrix (Distance)
_max = max(S)
while(_max >= T1){
createAssotiation(blob,object) //i,j of maximum
deleteCon&Row(blob,object)
_max = max(S)
}

rem_blob <- S;
rem_obj <-S;

FOR EACH rem_blob{
o = createNewObject(rem_blob)
objects.add(o)
}

FOR EACH rem_obj
rem_obj.ghostframe++;
if(rem_obj.ghostframe>GF)
delete(rem_obj,object)
}

}
*/