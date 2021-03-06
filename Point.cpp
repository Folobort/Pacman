#include <string>
#include <sstream>
#include <vector>

#include "Point.hpp"

using namespace std;

/// -CONSTRUCTORS-
Point::Point(unsigned x, unsigned y, unsigned w, unsigned h){
	coordX = x;
	coordY = y;
	width = w;
	height = h;
}
Point Point::copy(){
	return Point(coordX, coordY, width, height);
}

Point Point::next(){
	if(coordX == width-1){ // right side
		return Point(0, coordY+1, width, height);
	}
		
	return Point(coordX+1, coordY, width, height);
}
Point Point::before(){
	if(coordX == 0){
		return Point(width-1, coordY-1, width, height);
	}
	
	return Point(coordX-1, coordY, width, height);
}

Point Point::left(){
	if(coordX == 0){ // left side
		throw new exception();
	}
			
	return Point(coordX-1, coordY, width, height);
}
Point Point::right(){
	if(coordX == width-1){ // right side
		throw new exception();
	}
	
	return Point(coordX+1, coordY, width, height);
}
Point Point::up(){
	if(coordY == 0){ // top side
		throw new exception();
	}
		
	return Point(coordX, coordY-1, width, height);
}
Point Point::down(){
	if(coordY == height-1){ // bottom side
		throw new exception();
	}
		
	return Point(coordX, coordY+1, width, height);
}

/// -GETTERS-
unsigned Point::x(){
	return coordX;
}
unsigned Point::y(){
	return coordY;
}

unsigned Point::position(){
	return coordY*width + coordX;
}

/// -OTHERS-
bool Point::isLast(){
	return (coordX == width-1) && (coordY == height-1);
}

bool Point::equals(Point point){
	/// we assume we work on one grid
	return (coordX == point.x()) && (coordY == point.y());
}

bool Point::isInVector(vector<Point> S){
	for(unsigned i=0; i<S.size(); i++){
		if(equals(S[i])){
			return true;
		}
	}
	
	return false;
}

/// -TO STRING-
string Point::toString(){
	stringstream ss;
	ss << "(" << x() << "," << y() << ")";
	
	return ss.str();	
}







