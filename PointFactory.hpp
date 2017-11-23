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
	unsigned width;
	unsigned height;
	
	// PROTOTYPES
	public:
	PointFactory(unsigned w, unsigned h);
	
	Point mkPoint(unsigned x, unsigned y);
	
	Point mkPointLast();
};


#endif
