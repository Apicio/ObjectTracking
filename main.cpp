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

void displayTrackingResults(Mat, Mat, vector<t_tracks>, SystemObject*);

int main(int argc, char** argv)
{
	vector<t_tracks> tracks;
	int nextId = 1;
	SystemObject* obj = new SystemObject();
	namedWindow("Display window", WINDOW_AUTOSIZE); 

	for (int i = 1; i < 795; i++) {
		/** Declaration ************************************/
		t_Mat<int> bboxes, assignments, unassignedTracks, unassignedDetections;
		t_Mat<double> centroids;
		stringstream toOpen;
		Mat mask, frame;
		/** Declaration ************************************/
		toOpen << "view1/" << i << ".jpg";
		frame = imread(toOpen.str(), IMREAD_COLOR);

		obj->detectObjects(frame, /*return*/ centroids, bboxes, mask);
		obj->predictNewLocationsOfTracks(tracks);
		obj->detectionToTrackAssignment(tracks, centroids, /*return*/ assignments, unassignedTracks, unassignedDetections);
		obj->updateAssignedTracks(centroids, bboxes, assignments, /*return*/ tracks);
		obj->updateUnassignedTracks(unassignedDetections,  /*return*/ tracks);
		obj->deleteLostTracks(/*return*/ tracks);
		obj->createNewTracks(centroids, bboxes, unassignedDetections, /*return*/ nextId, tracks);

		displayTrackingResults(frame, mask, tracks, obj);
	}
	
	return 0;
}

void displayTrackingResults(Mat frame, Mat mask, vector<t_tracks> traks, SystemObject* obj) {
	imshow("Display window", frame);
	waitKey(0); // Wait for a keystroke in the window
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