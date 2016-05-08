#pragma once

#include <vector>
#include "Intestazione.h"
#define OUT_OF_BOUND -1


template <typename type> class t_Mat
{
private:
	type matrix[HEIGHT][WEIGHT];
	int row = 0;
	int col = 0;
public:
	t_Mat() {};
	void resize_matrix(int row_resize, int col_resize) {
		row = row_resize;
		col = col_resize;
	};
	type get(int i, int j) {
		if (i >= row || j >= col)
			return OUT_OF_BOUND;
		return matrix[i][j];
	};
	void set(int i, int j, type element) {
		int row_resize, col_resize;
		if (row < i + 1)
			row_resize = i + 1;
		if (col < j + 1)
			col_resize = j + 1;

		resize_matrix(row_resize, col_resize);
		matrix[i][j] = element;
	};
	bool isEmpty() {
		return row == 0 && col == 0;
	};
	int* getSize() {
		int s[2];
		s[0] = row;
		s[1] = col;
		return s;
	};
	int* maxSize() {
		int s[2];
		s[0] = HEIGHT;
		s[1] = WEIGHT;
		return s;
	};
	~t_Mat() {};
};