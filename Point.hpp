#ifndef POINT_H_
#define POINT_H_

#include <fstream>
#include <iostream>
#include <queue>
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <algorithm>

using namespace std;

class Point{
	// DATA
	unsigned coordX;
	unsigned coordY;
	unsigned width;
	unsigned height;
	
	// PROTOTYPES
	public:
	Point(unsigned x, unsigned y, unsigned w, unsigned h);
	
	unsigned x();
	unsigned y();
	
	unsigned position();
	
	Point next();
	Point before();
	
	Point left();
	Point right();
	Point up();
	Point down();
	
	bool isLast();
	
	Point copy();
	
	bool equals(Point point);
	
	bool isInVector(vector<Point> S);
	
	string toString();
};


#endif
