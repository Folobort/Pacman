#ifndef POINTFACTORY_H_
#define POINTFACTORY_H_

#include <vector>

#include "Point.hpp"

using namespace std;

class PointFactory{
	// == DATA ==
	vector<vector<bool>> matrix; 	// Matrix of the Point state: True for empty, False for wall
	
	// == PROTOTYPES ==
	public:
	
	/// -CONSTRUCTOR-
	PointFactory(vector<vector<bool>> m);
	
	/// -POINT CONSTRUCTORS-
	Point mkPoint(unsigned x, unsigned y);
	Point mkPointLast();					// Returns the bottom-right point of the grid
};


#endif
