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
	
	Point right(unsigned width){
		unsigned newX = coordX+1;
		unsigned newY = coordY;
		
		if(newX == width){
			newX = 0;
			newY++;
		}
		
		return Point(newX, newY);
	}
	
	bool last(unsigned width, unsigned height){
		return (coordX == width-1) && (coordY == height-1);
	}
	
	Point copy(){
		return Point(coordX, coordY);
	}
	
};


