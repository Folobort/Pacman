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


PointFactory::PointFactory(unsigned w, unsigned h){
	width = w;
	height = h;
}

Point PointFactory::mkPoint(unsigned x, unsigned y){
	return Point(x, y, width, height);
}

Point PointFactory::mkPointLast(){
	return Point(width-1, height-1, width, height);
}

