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
	
	// PROTOTYPES
	public:
	Kuplet(unsigned k);
	
	vector<Point> nextElement (vector<Point> element, unsigned width, unsigned height);
	
	vector<Point> firstElement (unsigned size, unsigned width, unsigned height);
	
	bool hasNoWall(vector<vector<bool>> matrix, vector<Point> element);

	bool isInMatrix(vector<vector<bool>> matrix, vector<Point> element);

};


#endif
