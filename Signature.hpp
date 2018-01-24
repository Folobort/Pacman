#ifndef SIGNATURE_H_
#define SIGNATURE_H_


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

#define STK_F 0
#define STK_C 1
#define STK_S 2


class Signature{
	// == DATA ==
	vector<Point> bag;			// Points in the signature
	vector<unsigned> stickers;	// And their state (Free, Covered, Selected)
	
	vector<Point> selected;		// Selected points that are no longer stored in the bag vector
	
	// == PROTOTYPES ==
	public:
	
	//CONSTRUCTORS
	Signature();
	Signature(vector<Point> bg);
	Signature copy();
	
	Signature updateBagWith(Point newPoint);
	void removeLastFromBag();
	
	// GETTERS
	int getIndex(Point p);
	unsigned getSticker(Point p);
	
	vector<Point> getBag();
	vector<unsigned> getStickers();
	vector<Point> getSelected();
	
	// SETTERS
	void setSticker(Point p, unsigned stk);
	void select(Point p);		/// pseudo-setter
	
	// COMPARATORS
	bool equals(Signature sig);
	bool isBetterFinalSig(Signature sig);
	
	// OTHERS
	vector<Signature> update(Point newPoint, vector<vector<vector<Point>>> neighbors);
	Signature updateF(Point newPoint, Point selected, vector<vector<vector<Point>>> neighbors);
	Signature updateS(Point newPoint, vector<vector<vector<Point>>> neighbors);
	Signature updateC(Point newPoint, vector<vector<vector<Point>>> neighbors);


	vector<Signature> update(vector<vector<vector<Point>>> neighbors);
	Signature updateF(Point selected, vector<vector<vector<Point>>> neighbors);
	Signature updateS(vector<vector<vector<Point>>> neighbors);
	Signature updateC(vector<vector<vector<Point>>> neighbors);
	
	// TO STRING
	string toString();

};



#endif
