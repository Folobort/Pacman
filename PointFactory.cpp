#include <fstream>
#include <iostream>
#include <queue>
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <algorithm>

#include "Point.hpp"
#include "PointFactory.hpp"

using namespace std;


PointFactory::PointFactory(vector<vector<bool>> m){
	matrix = m;
}

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

