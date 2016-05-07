#pragma once

#include <vector>
#define OUT_OF_BOUND -1

template <typename type> class t_Mat
{
private:
	vector< vector<type> > matrix(4, vector<int>(4));
	int row = 4;
	int col = 4;
public:
	void resize_matrix(int row_resize, int col_resize) {
		if(row_resize>row)
			this.matrix.resize(row_resize);

		if(col_resize>col)
			for (int i = 0; i < row_resize; i++)
				this.matrix.at(i).resize(col_resize);

		row = row_resize;
		col = col_resize;
	};
	type get(int i, int j) {
		if (i >= row || j >= col)
			return OUT_OF_BOUND;
		return matrix.at(i).at(j);
	};
	void set(int i, int j, type element) {
		int row_resize, col_resize;
		if (row < i + 1)
			row_resize = i + 1;
		if (col < j + 1)
			col_resize = j + 1;

		resize_matrix(row_resize, col_resize);
		this.matrix.at(i).at(j) = element;
	};
	int[2] getSize() {
		int s[2];
		s[0] = row;
		s[1] = col;
		return s;
	};
};

