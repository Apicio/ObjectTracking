#include "SystemObject.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/tracking.hpp"
#include <Windows.h>

using namespace cv;
using namespace std;

void SystemObject::detectObjects(const Mat frame, /*return*/ t_Mat<double>& centroids, t_Mat<int>& bboxes, Mat& mask)
{

}

void SystemObject::predictNewLocationsOfTracks(vector<t_tracks> tracks) {
	for (int i = 1; i < tracks.size; i++) {
		int* bbox = tracks.at(i).bbox;	/* Posizione nell'immagine [0] e [1] e dimensione del box [2] e [3] /
		int* predictedCentroid;			/* x, y */
		// Predict the current location of the track.
		predictedCentroid = KalmanFilter::predict(tracks.at(i).kalmaFilter);
		// Shift the bounding box so that its center is at the predicted location.
		predictedCentroid[0] = (int) (predictedCentroid[0] - bbox[2] / 2);
		predictedCentroid[1] = (int)(predictedCentroid[1] - bbox[3] / 2);
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
	int nTracks = tracks.size;
	int nDetections = centroids.getSize()[0]; /* Numero di riga rappresenta il numero di Blob individuati*/
	// Compute the cost of assigning each detection to each track.
	double SimilarityMatrix[HEIGHT + 1][WEIGHT + 1];

	// Inizializzazione riga/colonna aggiuntive per la cancellazione dei termini accoppati, nello scorrimento saranno saltate le righe/colonne che sono settate a 0.
	for (int i = 1; i < nDetections + 1; i++)
		SimilarityMatrix[i][0] = i;
	for (int j = 1; j < nTracks + 1; j++)
		SimilarityMatrix[0][j] = j;
	// Popolazione della matrice di Similarità  

	for (int i = 1; i < nTracks + 1; i++) {
		for (int j = 1; j < nDetections + 1; j++) {
			SimilarityMatrix[i][j] = distance(tracks(i).kalmanFilter, centroids);//??
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
		if (max > THRESHOLD) {
			// Salvo l'assegnazione
			assignments.set(0, num_of_assignments, max_i);
			assignments.set(1, num_of_assignments, max_j);
			num_of_assignments++;
			// Cancello le righe/colonne utilizzate
			SimilarityMatrix[max_i][0] = 0;
			SimilarityMatrix[max_j][0] = 0;
		}
	} while (max > THRESHOLD);

	// Verifico Blob non assegnati
	int num_of_unassignedTracks = 0;
	for (int i = 1; i < nDetections + 1; i++)
		if (SimilarityMatrix[i][0] == i) {
			unassignedTracks.set(0, num_of_unassignedTracks, i);
			num_of_unassignedTracks++;
		}

	// Verifico Oggetti non assegnati
	int num_of_unassignedDetections = 0;
	for (int j = 1; j < nTracks + 1; j++)
		if (SimilarityMatrix[0][j] == j) {
			unassignedDetections.set(0, num_of_unassignedTracks, j);
			num_of_unassignedDetections++;
		}
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
