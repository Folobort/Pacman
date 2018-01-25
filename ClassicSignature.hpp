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
#include "Bag.hpp"

using namespace std;

#define STK_F 0
#define STK_C 1
#define STK_S 2



class ClassicSignature{
	// == DATA ==
	Bag treeBag;
	
	vector<unsigned> stickers;	// And their state (Free, Covered, Selected)
	
	vector<unsigned> selectedID;		// Selected points that are no longer stored in the bag vector
	
	
	// == PROTOTYPES ==
	public:
	
	//CONSTRUCTORS
	ClassicSignature();
	ClassicSignature(Bag bg);
	ClassicSignature copy();
	
	// GETTERS
	int getIndex(unsigned id);
	unsigned getSticker(unsigned u);
	
	Bag getBag();
	vector<unsigned> getStickers();
	vector<unsigned> getSelectedID();
	
	unsigned getToCover();
	
	Node getNodeWithID(unsigned id, vector<Node> graph);
	
	// SETTERS
	void setSticker(unsigned id, unsigned stk);
	void select(unsigned idSelected);		/// pseudo-setter
	
	// COMPARATORS
	bool equals(ClassicSignature sig);
	bool isBetterFinalSig(ClassicSignature sig);
	
	// OTHERS
	
	bool isInBag(unsigned id);
	ClassicSignature sumSignature(vector<ClassicSignature>);
	
	void remove(unsigned id);


	vector<ClassicSignature> update(unsigned idToCover, vector<Node> graph);
	ClassicSignature updateF(unsigned idToCover, unsigned idSelected, vector<Node> graph);
	ClassicSignature updateS(unsigned idSelected, vector<Node> graph);
	ClassicSignature updateC(unsigned idCovered);
	
	// TO STRING
	string toString();

};



#endif
