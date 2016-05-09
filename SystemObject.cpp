#include "SystemObject.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#define CENTROIDDIM 2
#define BBOXDIM 4

using namespace cv;
using namespace std;

void SystemObject::detectObjects(const Mat frame, /*return*/ t_Mat<double>& centroids, t_Mat<int>& bboxes, Mat& mask)
{
	vector<KeyPoint> keypoints;  	
	Mat keyPointImage; Mat kernel; Mat labels; Mat centroid; Mat stats;


	/*Background Subtraction*/
	pMOG->apply(frame, mask);
	kernel = getStructuringElement(MORPH_ELLIPSE, Size(3, 3));
	morphologyEx(mask, mask, MORPH_OPEN, kernel);
	/*Blobs Detection, compute centroids and bboxes*/
	int nLabels = connectedComponentsWithStats(mask,labels, stats,centroid);
	
	int num_blob = 0;
	for (int i = 0; i < nLabels; i++) {
		float area = stats.at<int>(i, CC_STAT_AREA);
		if (area < MAXAREA && area > MINAREA) {
			centroids.set(num_blob, 0, centroid.at<double>(i, 0)); //x coord of centroid
			centroids.set(num_blob, 1, centroid.at<double>(i, 1)); //y coord of centroid

			/* bboxes organized as column matrix with 4 columns:

			| topmoseleft_corner | horizonal_size | vertical_size | area | */

 			bboxes.set(num_blob, 0, stats.at<int>(i, CC_STAT_LEFT)); //topmost left corner of bbox
			bboxes.set(num_blob, 1, stats.at<int>(i, CC_STAT_TOP));
			bboxes.set(num_blob, 2, stats.at<int>(i, CC_STAT_WIDTH)); //horizontal size of bbox
			bboxes.set(num_blob, 3, stats.at<int>(i, CC_STAT_HEIGHT)); //vertical size of bbox
			num_blob++;
		}
	}
	int a = 1;
}

void SystemObject::predictNewLocationsOfTracks(vector<t_tracks> tracks) {
	for (int i = 1; i < tracks.size(); i++) {
		int* bbox = tracks.at(i).bbox;	/* Posizione nell'immagine [0] e [1] e dimensione del box [2] e [3] */
		int predictedCentroid[2];		/* x, y */
		// Predict the current location of the track.
		Mat prediction = tracks.at(i).kalmanFilter.predict();
		predictedCentroid[0] = (int)(prediction.at<float>(0) - bbox[2] / 2);
		predictedCentroid[1] = (int)(prediction.at<float>(1) - bbox[3] / 2);
		int n_bbox[4] = {predictedCentroid[0], predictedCentroid[1], bbox[2], bbox[3]};
		tracks.at(i).bbox = n_bbox;
	}
}

/**
	@Params tracks: Oggetti già classificati al frame precedente
	@Params centroids: Centroidi degli oggetti ancora da classificare 
	@Return assignments: Corrispondenza tra Oggetto e Blob
	@Return unassignedTracks: Tracce non ancora assegnate, probabile Ghost
	@Return unassignedDetections: Blob non ancora assegnato, probabile nuovo oggetto nella scena
*/
void SystemObject::detectionToTrackAssignment(vector<t_tracks>tracks, t_Mat<double> centroids, /*return*/ t_Mat<int>& assignments, t_Mat<int>& unassignedTracks, t_Mat<int>& unassignedDetections)
{
	int nTracks = tracks.size();
	int nDetections = centroids.getSize()[0]; /* Numero di riga rappresenta il numero di Blob individuati*/
	// Compute the cost of assigning each detection to each track.
	double SimilarityMatrix[HEIGHT + 1][WEIGHT + 1];

	// Inizializzazione riga/colonna aggiuntive per la cancellazione dei termini accoppati, nello scorrimento saranno saltate le righe/colonne che sono settate a 0.
	for (int i = 1; i < nDetections + 1; i++)
		SimilarityMatrix[i][0] = i;
	for (int j = 1; j < nTracks + 1; j++)
		SimilarityMatrix[0][j] = j;
	// Popolazione della matrice di Similarità: calcolo misura di similarità
	for (int i = 1; i < nTracks + 1; i++) {
		for (int j = 1; j < nDetections + 1; j++) {

			Mat prediction = tracks.at(i).kalmanFilter.predict();
			int predictedCentroid[2];
			int centroid[2];
			predictedCentroid[0] = (int)(prediction.at<float>(0) - tracks.at(i).bbox[2] / 2);
			predictedCentroid[1] = (int)(prediction.at<float>(1) - tracks.at(i).bbox[3] / 2);
			centroid[0] = centroids.get(i, 0);
			centroid[1] = centroids.get(i, 1);
			float dist = sqrt(pow((predictedCentroid[0] - centroid[0]), 2) + pow((predictedCentroid[0] - centroid[0]), 2));
			dist = dist < DMAX ? dist : DMAX;
			SimilarityMatrix[i][j] = 1 - dist / DMAX;
		}
	}

	int max; int max_i; int max_j;
	int num_of_assignments = 0;
	do {
		max = 0; max_i = 0; max_j = 0;
		for (int i = 1; i < nTracks + 1; i++) {
			while (i < nTracks + 1 && SimilarityMatrix[i][0] == 0) {
				i++; // Salto le righe cancellate
			}
			for (int j = 1; j < nDetections + 1; j++) {
				while (j < nDetections + 1 && SimilarityMatrix[0][j] == 0) {
					j++; // Salto le colonne cancellate
				}
				if (i < nTracks + 1 && j < nDetections + 1 && SimilarityMatrix[i][j] > max) {
					max = SimilarityMatrix[i][j];
					max_i = i;
					max_j = j;
				}
			}				
		}
		int a = 1;
		if (max > THRESHOLD) {
			// Salvo l'assegnazione
			assignments.set(num_of_assignments, 0, max_i); /* Assign trackIdx */
			assignments.set(num_of_assignments, 1, max_j); /* Assign detectionIdx */
			num_of_assignments++;
			// Cancello le righe/colonne utilizzate
			SimilarityMatrix[max_i][0] = 0;
			SimilarityMatrix[max_j][0] = 0;
		}
	} while (max > THRESHOLD);

	// Verifico Blob non assegnati
	
	int num_of_unassignedDetections = 0;
	for (int i = 1; i < nDetections + 1; i++)
		if (SimilarityMatrix[i][0] == i) {
			unassignedDetections.set(num_of_unassignedDetections, 0, i);
			num_of_unassignedDetections++;
		}

	// Verifico Oggetti non assegnati
	int num_of_unassignedTracks = 0;
	for (int j = 1; j < nTracks + 1; j++)
		if (SimilarityMatrix[0][j] == j) {
			unassignedTracks.set(num_of_unassignedTracks, 0, j);
			num_of_unassignedTracks++;
		}
	int a = 1;
}
void SystemObject::updateAssignedTracks(t_Mat<double> centroids, t_Mat<int> bboxes, t_Mat<int> assignments, /*return*/ vector<t_tracks>& tracks)
{
	int numAssignedTracks = assignments.getSize()[1];
	for (int i = 0; i < numAssignedTracks; i++) {
		int trackIdx = assignments.get(0,i);
		int detectionIdx = assignments.get(1,i);
		int centroid[2]; /* x,y */
		centroid[0] = centroids.get(detectionIdx, 0);
		centroid[1] = centroids.get(detectionIdx, 1);
		int bbox[4]; /* x,y,w,h*/
		bbox[0] = bboxes.get(detectionIdx, 0);
		bbox[1] = bboxes.get(detectionIdx, 1);
		bbox[2] = bboxes.get(detectionIdx, 2);
		bbox[3] = bboxes.get(detectionIdx, 3);
		// Correct the estimate of the object's location using the new detection.
		Mat_<float> measurement(2, 1); measurement.setTo(Scalar(0));
		measurement(0) = centroid[0];
		measurement(1) = centroid[1];
		tracks.at(trackIdx).kalmanFilter.correct(measurement) ;
		// Replace predicted bounding box with detected bounding box.
		tracks.at(trackIdx).bbox = bbox;
		// Update track's age.
		tracks.at(trackIdx).age = tracks.at(trackIdx).age + 1;
		// Update visibility.
		tracks.at(trackIdx).totalVisibleCount = tracks.at(trackIdx).totalVisibleCount + 1;
		tracks.at(trackIdx).consecutiveInvisibleCount = 0;
	}
}
void SystemObject::updateUnassignedTracks(t_Mat<int> unassignedTracks,  /*return*/ vector<t_tracks>& tracks)
{
	int numUnassignedTracks = unassignedTracks.getSize()[0];
	if (numUnassignedTracks > tracks.size())
		return;

	for (int i = 0; i < numUnassignedTracks; i++) {
//		if (numUnassignedTracks > 0) { 
			int ind = unassignedTracks.get(0,i);
			tracks.at(ind).age = tracks.at(ind).age + 1;
			tracks.at(ind).consecutiveInvisibleCount = tracks.at(ind).consecutiveInvisibleCount + 1;
//		}
	}
}
void SystemObject::deleteLostTracks(/*return*/ vector<t_tracks> tracks)
{
	int numTraks = tracks.size();
	if (numTraks == 0)
		return;

	// Compute the fraction of the track's age for which it was visible.
	vector<int> ages, totalVisibleCounts;
	vector<double> visibility;
	for (int i = 0; i < numTraks; i++) {
		ages.push_back(tracks.at(i).age);
		totalVisibleCounts.push_back(tracks.at(i).totalVisibleCount);
		visibility.push_back(tracks.at(i).totalVisibleCount/tracks.at(i).age);
	}
	// Find the indices of 'lost' tracks.
	vector<int> lostInds;
	for (int i = 0; i < numTraks; i++) {
		if ((ages.at(i) < AGETHRESHOLD && visibility.at(i) < VISIBILITYTHRESHOLD) || (tracks.at(i).consecutiveInvisibleCount >= INVISIBLEFORTOOLONG)) {
			lostInds.push_back(i);
		}
	}

	// Delete lost tracks.
	for (int i = 0; i < lostInds.size(); i++) {
		tracks.erase(tracks.begin() + lostInds.at(i));
	}
}

t_Mat<double> getUnassignedCentroids(t_Mat<double> centroids, t_Mat<int> unassignedDetections) {
	t_Mat<double> unassignedDetections_centroids;
	for (int i = 0; i < unassignedDetections.getSize()[0]; i++) {
		for (int j = 0; j < CENTROIDDIM; j++) {
			unassignedDetections_centroids.set(i, j, centroids.get(unassignedDetections.get(0, j), j));
		}
	}
	return unassignedDetections_centroids;
}

t_Mat<int> getUnassignedBbox(t_Mat<double> centroids, t_Mat<int> unassignedDetections) {
	t_Mat<int> unassignedDetections_bboxes;
	for (int i = 0; i < unassignedDetections.getSize()[0]; i++) {
		for (int j = 0; j < BBOXDIM; j++) {
			unassignedDetections_bboxes.set(i, j, centroids.get(unassignedDetections.get(0, j), j));
		}
	}
	return unassignedDetections_bboxes;
}


void SystemObject::createNewTracks(t_Mat<double> centroids, t_Mat<int> bboxes, t_Mat<int> unassignedDetections, /*return*/ int& nextId, vector<t_tracks>& tracks)
{
	// Recuperiamo tutti i centroidi e bbox non assegnati 
	centroids = getUnassignedCentroids(centroids, unassignedDetections);
	bboxes = getUnassignedBbox(centroids, unassignedDetections);

	for (int i = 0; i < centroids.getSize()[0]; i++) {
		// Create a Kalman filter object.
		KalmanFilter kalmanFilter(4, 2, 0); /* Stato iniziale, posizione, velocità per x e y */
		kalmanFilter.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
		Mat_<float> measurement(2, 1); measurement.setTo(Scalar(0));

		kalmanFilter.statePre.at<float>(0) = centroids.get(i,0);
		kalmanFilter.statePre.at<float>(1) = centroids.get(i,1);
		kalmanFilter.statePre.at<float>(2) = 0;
		kalmanFilter.statePre.at<float>(3) = 0;

		setIdentity(kalmanFilter.measurementMatrix);
		setIdentity(kalmanFilter.processNoiseCov, Scalar::all(1e-4));
		setIdentity(kalmanFilter.measurementNoiseCov, Scalar::all(10));
		setIdentity(kalmanFilter.errorCovPost, Scalar::all(.1));

		// Create a new track
		t_tracks track;
		track.id = nextId;
		int temp_bbox[4] = {centroids.get(i,0), centroids.get(i,1), centroids.get(i,2), centroids.get(i,3)};
		track.bbox = temp_bbox;
		track.kalmanFilter = kalmanFilter;
		track.age = 1;
		track.totalVisibleCount = 1;
		track.consecutiveInvisibleCount = 0;
		// Add it to the array of tracks.
		tracks.push_back(track);
		// Increment the next id.
		nextId++;
	}
}

SystemObject::~SystemObject()
{
}
