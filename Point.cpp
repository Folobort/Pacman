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


Point::Point(unsigned x, unsigned y){
	coordX = x;
	coordY = y;
}

unsigned Point::x(){
	return coordX;
}
unsigned Point::y(){
	return coordY;
}

unsigned Point::position(unsigned width){
	return coordX*width + coordY;
}

Point Point::next(unsigned width){
	if(coordX == width-1){ // right side
		return Point(0, coordY+1);
	}
		
	return Point(coordX+1, coordY);
}

Point Point::left(){
	if(coordX == 0){ // left side
		throw new exception();
	}
			
	return Point(coordX-1, coordY);
}
Point Point::right(unsigned width){
	if(coordX == width-1){ // right side
		throw new exception();
	}
		
	return Point(coordX+1, coordY);
}
Point Point::up(){
	if(coordY == 0){ // top side
		throw new exception();
	}
		
	return Point(coordX, coordY-1);
}
Point Point::down(unsigned height){
	if(coordY == height-1){ // bottom side
		throw new exception();
	}
		
	return Point(coordX, coordY+1);
}

bool Point::isLast(unsigned width, unsigned height){
	return (coordX == width-1) && (coordY == height-1);
}

Point Point::copy(){
	return Point(coordX, coordY);
}

bool Point::equals(Point point){
	return (coordX == point.x()) && (coordY == point.y());
}

