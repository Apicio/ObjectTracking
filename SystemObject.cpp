#include "SystemObject.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#define CENTROIDDIM 2
#define BBOXDIM 4
#define DELETED 0

using namespace cv;
using namespace std;

void SystemObject::detectObjects(const Mat frame, /*return*/ vector<double*>& centroids, vector<double*>& bboxes, Mat& mask)
{
	vector<KeyPoint> keypoints;  	
	Mat keyPointImage; Mat kernel; Mat labels; Mat centroid; Mat stats;


	/*Background Subtraction*/
	pMOG->apply(frame, mask);
	kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
	morphologyEx(mask, mask, MORPH_OPEN, kernel); // Opening to remove spikes and noise (erode + dilate)
	kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
	morphologyEx(mask, mask, MORPH_CLOSE, kernel); // Closing to fill holes (dilate + erode)
	/*Blobs Detection, compute centroids and bboxes*/
	int nLabels = connectedComponentsWithStats(mask,labels, stats,centroid);
	
	int num_blob = 0;
	for (int i = 0; i < nLabels; i++) {
		float area = stats.at<int>(i, CC_STAT_AREA);
		if (area < MAXAREA && area > MINAREA) {
			double* cent = new  double[2];
			cent[0] = centroid.at<double>(i, 0);
			cent[1] = centroid.at<double>(i, 1);
			centroids.push_back(cent); //x,y coord of centroid
			double* box = new double[4];
			box[0] = stats.at<int>(i, CC_STAT_LEFT); /*topmost left corner of bbox*/
			box[1] = stats.at<int>(i, CC_STAT_TOP);
			box[2] = stats.at<int>(i, CC_STAT_WIDTH);
			box[3] = stats.at<int>(i, CC_STAT_HEIGHT);
			bboxes.push_back(box);
		}
	}
}

void SystemObject::predictNewLocationsOfTracks(vector<t_tracks>& tracks) {
	for (int i = 1; i < tracks.size(); i++) {
		double* bbox = tracks.at(i).bbox;	/* Posizione nell'immagine [0] e [1] e dimensione del box [2] e [3] */
		double predictedCentroid[2];		/* x, y */
		// Predict the current location of the track.
		Mat prediction = tracks.at(i).kalmanFilter.predict();
		predictedCentroid[0] = (int)(prediction.at<float>(0) - bbox[2] / 2);
		predictedCentroid[1] = (int)(prediction.at<float>(1) - bbox[3] / 2);
		double* n_bbox = new double[4];
		n_bbox[0] = predictedCentroid[0];
		n_bbox[1] = predictedCentroid[1];
		n_bbox[2] = bbox[2];
		n_bbox[3] = bbox[3];
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
void SystemObject::detectionToTrackAssignment(vector<t_tracks>tracks, vector<double*> centroids, /*return*/ vector<int*>& assignments, vector<int>& unassignedTracks, vector<int>& unassignedDetections) 
{
	int nTracks = tracks.size();
	int nDetections = centroids.size(); /* Numero di riga rappresenta il numero di Blob individuati*/
	// Compute the cost of assigning each detection to each track.
	double SimilarityMatrix[HEIGHT + 1][WEIGHT + 1];

	// Inizializzazione riga/colonna aggiuntive per la cancellazione dei termini accoppati, nello scorrimento saranno saltate le righe/colonne che sono settate a 0.
	for (int i = 1; i < nDetections + 1; i++)
		SimilarityMatrix[i][0] = i;
	for (int j = 1; j < nTracks + 1; j++)
		SimilarityMatrix[0][j] = j;
	// Popolazione della matrice di Similarità: calcolo misura di similarità
	for (int i = 1; i < nDetections + 1; i++) {
		for (int j = 1; j < nTracks + 1; j++) {

		/*	Mat prediction = tracks.at(j-1).kalmanFilter.predict(); // non predice posizione
			int predictedCentroid[2];
			predictedCentroid[0] = (double)(prediction.at<float>(0) - tracks.at(j-1).bbox[2] / 2);
			predictedCentroid[1] = (double)(prediction.at<float>(1) - tracks.at(j-1).bbox[3] / 2);*/
			//Ricavo centroide dalla traccia
			int width = tracks.at(j-1).bbox[3];
			int heigh = tracks.at(j-1).bbox[2];
			int y = tracks.at(j-1).bbox[0];
			int x = tracks.at(j-1).bbox[1];
			double currCent[2] = {  x + width / 2,y + heigh / 2 };


			float dist = sqrt(pow(currCent[0] - centroids.at(i - 1)[0],/*^*/2) + pow(currCent[1] - centroids.at(i - 1)[1],/*^*/2));
			dist = dist < DMAX ? dist : DMAX;
			SimilarityMatrix[i][j] = 1 - dist / DMAX;
		}
	}

	int max; int max_i=1; int max_j=1;
	int num_of_assignments = 0;
	do {
		max = 0; max_i = 1; max_j = 1;
		for (int i = 1; i < nDetections + 1; i++) {
			while (i < nDetections + 1 && SimilarityMatrix[i][0] == DELETED) {
				i++; // Salto le righe cancellate
			}
			for (int j = 1; j < nTracks + 1; j++) {
				while (j < nTracks + 1 && SimilarityMatrix[0][j] == DELETED) {
					j++; // Salto le colonne cancellate
				}
				if (i < nDetections + 1 && j < nTracks + 1 && SimilarityMatrix[i][j] > max) {
					max = SimilarityMatrix[i][j];
					max_i = i;
					max_j = j;
				}
			}				
		}
		if (max > THRESHOLD) {
			// Salvo l'assegnazione
			int* ax = new int[2];
			ax[0] = max_i - 1; /* Assign trackIdx */
			ax[1] = max_j - 1; /* Assign detectionIdx */
			assignments.push_back(ax);
			// Cancello le righe/colonne utilizzate
			SimilarityMatrix[max_i][0] = DELETED;
			SimilarityMatrix[0][max_j] = DELETED;
		}
	} while (max > THRESHOLD);

	// Verifico Blob non assegnati
	for (int i = 1; i < nDetections + 1; i++)
		if (SimilarityMatrix[i][0] != DELETED) {
			unassignedDetections.push_back(i-1);
		}

	// Verifico Oggetti non assegnati
	for (int j = 1; j < nTracks + 1; j++)
		if (SimilarityMatrix[0][j] != DELETED) {
			unassignedTracks.push_back(j-1);
		}
}
void SystemObject::updateAssignedTracks(vector<double*> centroids, vector<double*> bboxes, vector<int*> assignments, /*return*/ vector<t_tracks>& tracks) 
{
	int numAssignedTracks = assignments.size();
	for (int i = 0; i < numAssignedTracks; i++) {
		int trackIdx = assignments.at(i)[0];
		int detectionIdx = assignments.at(i)[1];
		// Correct the estimate of the object's location using the new detection.
		Mat_<float> measurement(2, 1); measurement.setTo(Scalar(0));
		measurement(0) = centroids.at(detectionIdx)[0];
		measurement(1) = centroids.at(detectionIdx)[1];
		tracks.at(trackIdx).kalmanFilter.correct(measurement) ;
		// Replace predicted bounding box with detected bounding box.
		double* bbox = new double[4]; /* x,y,w,h*/
		bbox[0] = bboxes.at(detectionIdx)[0];
		bbox[1] = bboxes.at(detectionIdx)[1];
		bbox[2] = bboxes.at(detectionIdx)[2];
		bbox[3] = bboxes.at(detectionIdx)[3];
		tracks.at(trackIdx).bbox = bbox;
		// Update track's age.
		tracks.at(trackIdx).age = tracks.at(trackIdx).age + 1;
		// Update visibility.
		tracks.at(trackIdx).totalVisibleCount = tracks.at(trackIdx).totalVisibleCount + 1;
		tracks.at(trackIdx).consecutiveInvisibleCount = 0;
	}
}
void SystemObject::updateUnassignedTracks(vector<int> unassignedTracks,  /*return*/ vector<t_tracks>& tracks) 
{
	int numUnassignedTracks = unassignedTracks.size();
	if (numUnassignedTracks > tracks.size())
		return;

	for (int i = 0; i < numUnassignedTracks; i++) {
			int ind = unassignedTracks.at(i);
			tracks.at(ind).age = tracks.at(ind).age + 1;
			tracks.at(ind).consecutiveInvisibleCount = tracks.at(ind).consecutiveInvisibleCount + 1;
	}
}
void SystemObject::deleteLostTracks(/*return*/ vector<t_tracks>& tracks)
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
		if (!((ages.at(i) < AGETHRESHOLD && visibility.at(i) < VISIBILITYTHRESHOLD) || (tracks.at(i).consecutiveInvisibleCount >= INVISIBLEFORTOOLONG))) {
			lostInds.push_back(i);
		}
	}
	 
	// Delete lost tracks.
	for (int i = lostInds.size() - 1; i >= 0; i--) {
		tracks.erase(tracks.begin() + lostInds.at(i));
	}

}

vector<double*> getUnassignedCentroids(vector<double*> centroids, vector<int> unassignedDetections) {
	vector<double*> unassignedDetections_centroids;
	for (int i = 0; i < unassignedDetections.size(); i++) {
		double* cent = new double[2];
		cent[0] = centroids.at(unassignedDetections.at(i))[0];
		cent[1] = centroids.at(unassignedDetections.at(i))[1];
		unassignedDetections_centroids.push_back(cent);
	}
	return unassignedDetections_centroids;
}

vector<double*> getUnassignedBbox(vector<double*> bboxes, vector<int> unassignedDetections) {
	vector<double*> unassignedDetections_bboxes;
	for (int i = 0; i < unassignedDetections.size(); i++) {
		double* box = new double[4];
		box[0] = bboxes.at(unassignedDetections.at(i))[0];
		box[1] = bboxes.at(unassignedDetections.at(i))[1];
		box[2] = bboxes.at(unassignedDetections.at(i))[2];
		box[3] = bboxes.at(unassignedDetections.at(i))[3];
		unassignedDetections_bboxes.push_back(box);
	}
	return unassignedDetections_bboxes;
}

void SystemObject::createNewTracks(vector<double*> centroids, vector<double*> bboxes, vector<int> unassignedDetections, /*return*/ int& nextId, vector<t_tracks>& tracks) {
	// Recuperiamo tutti i centroidi e bbox non assegnati 
	centroids = getUnassignedCentroids(centroids, unassignedDetections);
	bboxes = getUnassignedBbox(bboxes, unassignedDetections);

	for (int i = 0; i < unassignedDetections.size(); i++) {
		// Create a Kalman filter object.
		KalmanFilter kalmanFilter(4, 2, 0); /* Stato iniziale, posizione, velocità per x e y */
		kalmanFilter.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
		Mat_<float> measurement(2, 1); measurement.setTo(Scalar(0));

		kalmanFilter.statePre.at<float>(0) = centroids.at(i)[0];
		kalmanFilter.statePre.at<float>(1) = centroids.at(i)[1];
		kalmanFilter.statePre.at<float>(2) = 0;
		kalmanFilter.statePre.at<float>(3) = 0;

		setIdentity(kalmanFilter.measurementMatrix);
		setIdentity(kalmanFilter.processNoiseCov, Scalar::all(1e-4));
		setIdentity(kalmanFilter.measurementNoiseCov, Scalar::all(10));
		setIdentity(kalmanFilter.errorCovPost, Scalar::all(.1));

		// Create a new track
		t_tracks track;
		track.id = nextId;
		double* temp_bbox = new double[4];
		temp_bbox[0] = bboxes.at(i)[0];
		temp_bbox[1] = bboxes.at(i)[1];
		temp_bbox[2] = bboxes.at(i)[2];
		temp_bbox[3] = bboxes.at(i)[3];
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