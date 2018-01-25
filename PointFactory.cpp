#include <vector>

#include "Point.hpp"
#include "PointFactory.hpp"

using namespace std;

/// -CONSTRUCTOR-
PointFactory::PointFactory(vector<vector<bool>> m){
	matrix = m;
}

/// -POINT CONSTRUCTORS-
Point PointFactory::mkPoint(unsigned x, unsigned y){
	unsigned w = matrix.size();
	unsigned h = matrix[0].size();
	
	return Point(x, y, w, h);
}

Point PointFactory::mkPointLast(){
	unsigned w = matrix.size();
	unsigned h = matrix[0].size();
	return mkPoint(w-1, h-1);
}

