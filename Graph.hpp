#ifndef GRAPH_H_
#define GRAPH_H_

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

const unsigned INFINITY = -1;

class Graph{
	// DATA
	vector<vector<bool>> matrix; // False for wall
	vector<vector<bool>> processed;
	vector<vector<unsigned>> dist;
	unsigned INFINITY;
	
	// PROTOTYPES
	public:
	Graph();
	
	void setMatrix(vector<vector<bool>> matrix);
	
	bool isWall(unsigned x, unsigned y);
	
	bool isDominatedBy(vector<Point> tuple);
	
	unsigned makeDijkstra(vector<Point> tuple);
	
	unsigned bestDijkstra(unsigned k);
};


#endif
