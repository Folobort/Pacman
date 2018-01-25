#ifndef CLASSICSIGNATURE_H_
#define CLASSICSIGNATURE_H_


#include <fstream>
#include <iostream>
#include <queue>
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <algorithm>

#include "Node.hpp"
#include "TreeDec.hpp"
#include "TreeNode.hpp"
#include "Signature.hpp"
#include "GraphAux.hpp"

using namespace std;

#define STK_F 0
#define STK_C 1
#define STK_S 2


class Signature{
	// == DATA ==
	vector<unsigned> stickers;	// And their state (Free, Covered, Selected)
	
	vector<unsigned> selectedID;		// Selected points that are no longer stored in the bag vector
	
	// == PROTOTYPES ==
	public:
	
	//CONSTRUCTORS
	Signature();
	Signature(TreeNode bg);
	Signature copy();
	
	Signature updateBagWith(Point newPoint);
	void removeLastFromBag();
	
	// GETTERS
	int getIndex(unsigned u);
	unsigned getSticker(unsigned u);
	
	vector<unsigned> getBag();
	vector<unsigned> getStickers();
	vector<unsigned> getSelected();
	
	// SETTERS
	void setSticker(unsigned u, unsigned stk);
	void select(unsigned u);		/// pseudo-setter
	
	// COMPARATORS
	bool equals(Signature sig);
	bool isBetterFinalSig(Signature sig);
	
	// OTHERS
	vector<Signature> update(Point newPoint, vector<vector<vector<Point>>> neighbors);
	Signature updateF(Point newPoint, Point selected, vector<vector<vector<Point>>> neighbors);
	Signature updateS(Point newPoint, vector<vector<vector<Point>>> neighbors);
	Signature updateC(Point newPoint, vector<vector<vector<Point>>> neighbors);
	
	
	Signature sum( Signature sig1; Signature Sig2);


	vector<Signature> update(vector<vector<vector<Point>>> neighbors);
	Signature updateF(Point selected, vector<vector<vector<Point>>> neighbors);
	Signature updateS(vector<vector<vector<Point>>> neighbors);
	Signature updateC(vector<vector<vector<Point>>> neighbors);
	
	// TO STRING
	string toString();

};



#endif
