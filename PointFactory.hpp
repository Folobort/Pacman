#ifndef POINTFACTORY_H_
#define POINTFACTORY_H_

#include <fstream>
#include <iostream>
#include <queue>
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <algorithm>

#include "Point.hpp"

using namespace std;

class PointFactory{
	// DATA
	vector<vector<bool>> matrix; // false for wall
	
	// PROTOTYPES
	public:
	PointFactory(vector<vector<bool>> m);
	
	Point mkPoint(unsigned x, unsigned y);
	
	Point mkPointLast();
};


#endif
