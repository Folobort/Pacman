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
	
	// PROTOTYPES
	public:
	Point(unsigned x, unsigned y);
	
	unsigned x();
	unsigned y();
	
	unsigned position(unsigned width);
	
	Point next(unsigned width);
	
	Point left();
	Point right(unsigned width);
	Point up();
	Point down(unsigned height);
	
	bool isLast(unsigned width, unsigned height);
	
	Point copy();
	
	bool equals(Point point);
	
	bool isInVector(vector<Point> S);
	
	string toString();
};


#endif
