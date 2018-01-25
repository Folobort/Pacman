#ifndef KUPLET_H_
#define KUPLET_H_

#include <vector>

using namespace std;

#include "Point.hpp"

class Kuplet{
	// == DATA ==
	unsigned k;
	
	unsigned width;
	unsigned height;
	
	// == PROTOTYPES ==
	public:
	
	/// -CONSTRUCTOR-
	Kuplet(unsigned k, unsigned w, unsigned h);
	
	/// -TUPLE CONSTRUCTORS-
	vector<Point> firstElement (unsigned size);			// Creates a tuple of defined size, with point of least position values (ie, top-leftest bag)
	vector<Point> nextElement (vector<Point> element);	// Returns the tuple that follows the parameter (according our strict ordering, that is a shift to the right)
	
	/// -OTHERS-
	bool hasNoWall(vector<vector<bool>> matrix, vector<Point> element);		// Returns False if any point of the tuple is in a wall state, True otherwise
	bool isInMatrix(vector<vector<bool>> matrix, vector<Point> element);	// Checks if all points of the tuple are refering to valid coordinates in the grid

};


#endif
