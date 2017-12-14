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
	// == DATA ==
	unsigned coordX;	/// Coordinate on the horizontal axis of the grid
	unsigned coordY;	/// Coordinate on the vertical axis of the grid
	
	unsigned width;		/// Grid width
	unsigned height;	/// Grid Height
	
	// == PROTOTYPES ==
	public:
	
	// CONSTRUCTORS
	Point(unsigned x, unsigned y, unsigned w, unsigned h);
	Point copy();
	
	Point next();	/// The point with position +1 (not always right)
	Point before();	/// The point with position -1 (not always left)
	Point left();	/// As the name implies. Throws an exception if the point is the the wrong edge and no valid point can be returned.
	Point right();	/// ...
	Point up();		/// ...
	Point down();	/// ...
	
	// GETTERS
	unsigned x();
	unsigned y();
	unsigned position();	/// Unique number associated to coordinates
	
	// OTHERS
	bool isLast();						/// True if the point is in the bottom-right corner
	bool equals(Point point);			/// True if the two points have the same coordinates
	bool isInVector(vector<Point> S);
	
	// TO STRING
	string toString();
};


#endif
