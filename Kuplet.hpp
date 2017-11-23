#ifndef KUPLET_H_
#define KUPLET_H_

#include <fstream>
#include <iostream>
#include <queue>
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <algorithm>

using namespace std;

#include "Point.hpp"

class Kuplet{
	// DATA
	unsigned k;
	unsigned width;
	unsigned height;
	
	// PROTOTYPES
	public:
	Kuplet(unsigned k, unsigned w, unsigned h);
	
	vector<Point> nextElement (vector<Point> element);
	
	vector<Point> firstElement (unsigned size);
	
	bool hasNoWall(vector<vector<bool>> matrix, vector<Point> element);

	bool isInMatrix(vector<vector<bool>> matrix, vector<Point> element);

};


#endif
