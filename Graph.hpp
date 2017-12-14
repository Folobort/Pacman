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
#include "PointFactory.hpp"
#include "Signature.hpp"

using namespace std;

const unsigned INFINITY = -1;

class Graph{
	// == DATA ==
	vector<vector<bool>> matrix; // False for wall
	vector<vector<vector<Point>>> neighbors;
	vector<vector<bool>> processed;
	vector<vector<unsigned>> dist;
	PointFactory pf = PointFactory(vector<vector<bool>>());

	//Tree decomposition
	vector<vector<Point>> tree;
	vector<Signature> sigSet;
	unsigned INFINITY;

	// == PROTOTYPES ==
	public:
	
	// CONSTRUCTORS
	Graph();
	void setMatrix(vector<vector<bool>> matrix);


	void treeDecomposition();
	vector<Point> firstBag();
	vector<Point> nextBag(vector<Point> bag);
	Point lastNoWall();


	bool isWall(unsigned x, unsigned y);
	bool isWall(Point p);

	bool isDominatedBy(vector<Point> tuple);

	unsigned makeDijkstra(vector<Point> tuple);
	unsigned bestDijkstra(unsigned k);

	unsigned degreeOf(Point point);

	Point getSmallDegreeUndominated(vector<Point> S);
	vector<Point> getNeighbors(Point point);
	vector<Point> getNeighborsUndominating(Point point, vector<Point> S);

	vector<Point> k_dominant(unsigned k, vector<Point> S);
	
	string toString();
	
	
	vector<Signature> cleanMultipleSig(vector<Signature> sigs);
	vector<Signature> nextSigSet(Point newPoint);
	vector<Signature> nextSigSet();
	
	void firstSigSet();
	
	void wololoLoop();
	
	Point diff(vector<Point> vp1, vector<Point> vp2);
	
};


#endif
