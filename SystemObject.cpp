#include "SystemObject.h"


SystemObject::SystemObject()
{
}

void SystemObject::detectObjects(const Mat frame, /*return*/ t_Mat<double>& centroids, t_Mat<int>& bboxes, Mat& mask)
{

}

void SystemObject::predictNewLocationsOfTracks(vector<t_tracks>) {

}

void SystemObject::detectionToTrackAssignment(vector<t_tracks>tracks, t_Mat<double> centroids, /*return*/ t_Mat<int>& assignments, t_Mat<int>& unassignedTracks, t_Mat<int>& unassignedDetections)
{

}
void SystemObject::updateAssignedTracks(t_Mat<double> centroids, t_Mat<int> bboxes, t_Mat<int> assignments, /*return*/ vector<t_tracks>& tracks)
{

}
void SystemObject::updateUnassignedTracks(t_Mat<int> unassignedDetections,  /*return*/ vector<t_tracks>& tracks)
{

}
void SystemObject::deleteLostTracks(/*return*/ vector<t_tracks> tracks)
{

}
void SystemObject::createNewTracks(t_Mat<double> centroids, t_Mat<int> bboxes, t_Mat<int> unassignedDetections, /*return*/ int& nextId, vector<t_tracks>& tracks)
{

}

SystemObject::~SystemObject()
{
}
