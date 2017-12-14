#include <fstream>
#include <iostream>
#include <queue>
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <algorithm>

#include "Signature.hpp"

using namespace std;

// CONSTRUCTORS
Signature::Signature(){}

Signature::Signature(vector<Point> bg){
	bag = bg;
	stickers = vector<unsigned>(bag.size(),STK_F);
}

Signature Signature::copy(){
	Signature newSig;
	
	newSig.bag = bag;
	newSig.stickers = stickers;
	newSig.selected = selected;
	
	return newSig;
}

Signature Signature::updateBagWith(Point newPoint){
	Signature newSig;
	
	// Equivalent to a bag.push_front(newPoint); stickers.push_front(STK_F);
	newSig.bag.push_back(newPoint);
	newSig.stickers.push_back(STK_F);
	for(unsigned index=0; index<bag.size(); index++){
		newSig.bag.push_back(bag[index]);
		newSig.stickers.push_back(stickers[index]);
	}
	
	// Copy of selected
	newSig.selected = selected;
	
	// remove last point now (does update selected if necessary)
	newSig.removeLastFromBag();
	
	return newSig;
}

Signature Signature::removeLastFromBag(){
	Signature newSig = copy();
	
	if(stickers[stickers.size()-1] == STK_S){
		newSig.selected.push_back(bag[bag.size()-1]);
	}
	bag.pop_back();
	stickers.pop_back();
	
	return newSig;
}


// GETTERS
int Signature::getIndex(Point p){
	for(unsigned i=0; i<bag.size(); i++){
		if(bag[i].equals(p)){
			return i;
		}
	}
	// Not in Signature List
	return -1;
}

unsigned Signature::getSticker(Point p){
	int index = getIndex(p);
	if(index != -1){
		return stickers[index];
	} else{
		cout << "OLALA KESKISPASS SIGNATURE::GETSTICKER" << endl;
		return 23;
	}
}

vector<Point> Signature::getBag(){
	return bag;
}

vector<unsigned> Signature::getStickers(){
	return stickers;
}

vector<Point> Signature::getSelected(){
	return selected;
}
	

// SETTERS
void Signature::setSticker(Point p, unsigned stk){
	int index = getIndex(p);
	
	if(index != -1){
		stickers[index] = stk;
	} else{
		cout << "OLALA KESKISPASS SIGNATURE::SETSTICKER" << endl;
	}
}

void Signature::select(Point p){
	selected.push_back(p);
}


// COMPARATORS
bool Signature::equals(Signature sig){ // only with same bags (ie, same node of the tree dec)
	return equal(stickers.begin(), stickers.end(), sig.stickers.begin());
}


// OTHERS
/// It is a bit annoying to carry the neighbor triple vector around. The other way would be to store it in every Point, but it would take its toll space-wise.
vector<Signature> Signature::update(Point newPoint, vector<vector<vector<Point>>> neighbors){
	vector<Signature> newSigs;
	Signature newSig;

	// We take the point to be removed next, and its neighbors in the graph.
	Point toCover = bag.back();
	unsigned stk = getSticker(toCover);
	vector<Point> nb = neighbors[toCover.x()][toCover.y()];
	
	if(stk == STK_F){ /// Free (not covered): we need to select this point or a neighbor (so deg+1 new signatures)
		for(unsigned i=0; i<nb.size(); i++){ // select a neighbor
			if(nb[i].isInVector(bag)){
				newSig = updateF(newPoint, nb[i], neighbors);
				newSigs.push_back(newSig);
			}
		}
		
		// select point: same as (stk = STK_S)
		newSig = updateS(newPoint, neighbors);
		newSigs.push_back(newSig);
	}
	else if(stk == STK_C){ /// Covered: nothing to do
		newSig = updateC(newPoint, neighbors);
		newSigs.push_back(newSig);
	}
	else{ /// stk == STK_S, Selected: count+1 and set neighbors to covered
		newSig = updateS(newPoint, neighbors);
		newSigs.push_back(newSig);
	}
	
	return newSigs;
}

Signature Signature::updateF(Point newPoint, Point selected, vector<vector<vector<Point>>> neighbors){
	Signature newSig = updateBagWith(newPoint);
	
	// set the point to selected
	newSig.setSticker(selected, STK_S);
	
	// each neighbor which is not covered (STK_F) becomes covered (STK_C)
	vector<Point> nb = neighbors[selected.x()][selected.y()];
	for(unsigned i=0; i<nb.size(); i++){
		int index = newSig.getIndex(nb[i]); /// (index = -1) in case of a wall
		if(index != -1 && newSig.stickers[index] == STK_F){ // a non-wall non-covered neighbor
			newSig.stickers[index] = STK_C;
		}
	}
	
	return newSig;
}

Signature Signature::updateS(Point newPoint, vector<vector<vector<Point>>> neighbors){
	// This case (last point of the bag with an STK_S) is equivalent to finding this point with STK_F and setting it to selected now.
	// Moreover, it helps making sure all its neighbors are set to STK_C. (which might not be the case of the higher/lefter ones)
	Point selected = bag.back();
	Signature newSig = updateF(newPoint, selected, neighbors);
	
	return newSig;
}

Signature Signature::updateC(Point newPoint, vector<vector<vector<Point>>> neighbors){
	// Nothing to do in this case but update the bag
	Signature newSig = updateBagWith(newPoint);
	
	return newSig;
}







