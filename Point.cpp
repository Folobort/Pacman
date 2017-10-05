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
	private:
	unsigned coordX;
	unsigned coordY;
	
	public:
	Point(unsigned x, unsigned y){
		coordX = x;
		coordY = y;
	}
	
	unsigned x(){
		return coordX;
	}
	unsigned y(){
		return coordY;
	}
	
	unsigned position(unsigned width){
		return coordX*width + coordY;
	}
	
	Point next(unsigned width){
		if(coordX == width-1){ // right side
			return Point(0, coordY+1);
		}
		
		return Point(coordX+1, coordY);
	}
	
	Point left(){
		if(coordX == 0){ // left side
			throw new exception();
		}
		
		return Point(coordX-1, coordY);
	}
	Point right(unsigned width){
		if(coordX == width-1){ // right side
			throw new exception();
		}
		
		return Point(coordX+1, coordY);
	}
	Point up(){
		if(coordY == 0){ // top side
			throw new exception();
		}
		
		return Point(coordX, coordY-1);
	}
	Point down(unsigned height){
		if(coordY == height-1){ // left side
			throw new exception();
		}
		
		return Point(coordX, coordY+1);
	}
	
	
	bool last(unsigned width, unsigned height){
		return (coordX == width-1) && (coordY == height-1);
	}
	
	Point copy(){
		return Point(coordX, coordY);
	}
	
	bool equals(Point point){
		return (x == point.x()) && (y == point.y());
	}
	
};


