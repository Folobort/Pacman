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

void Signature::removeLastFromBag(){
	bag.pop_back();
	stickers.pop_back();
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
	}/// In fact, the else case happens and does not matter
}

void Signature::select(Point p){
	selected.push_back(p);
}


// COMPARATORS
bool Signature::equals(Signature sig){ // only with same bags (ie, same node of the tree dec)
	return equal(stickers.begin(), stickers.end(), sig.stickers.begin());
}

bool Signature::isBetterFinalSig(Signature sig){
	return (getSelected().size() <= sig.getSelected().size());
}


// OTHERS
/// It is a bit annoying to carry the neighbor triple vector around. The other way would be to store it in every Point, but it would take its toll space-wise.
vector<Signature> Signature::update(Point newPoint, vector<vector<vector<Point>>> neighbors){
	vector<Signature> newSigs;
	Signature newSig;
/*
	cout << "===========" << endl;
	cout << "Point :" << endl;
	cout << newPoint.toString() << endl;
	cout << "===========" << endl;
	

	cout << "===========" << endl;
	cout << "Sig init :" << endl;
	cout << toString() << endl;
	cout << "===========" << endl;
*/

	// We take the point to be removed next, and its neighbors in the graph.
	Point toCover = bag.back();
/*
	cout << "===========" << endl;
	cout << "PointTC :" << endl;
	cout << toCover.toString() << endl;
	cout << "===========" << endl;
*/
	unsigned stk = getSticker(toCover);
	vector<Point> nb = neighbors[toCover.x()][toCover.y()];
	
	if(stk == STK_F){ /// Free (not covered): we need to select this point or a neighbor (so deg+1 new signatures)
		for(unsigned i=0; i<nb.size(); i++){ // select a neighbor
			if(nb[i].isInVector(bag)){
				newSig = updateF(newPoint, nb[i], neighbors);
				/*
				cout << "===========" << endl;
				cout << "Sig added :" << endl;
				cout << newSig.toString() << endl;
				cout << "===========" << endl;
				*/
				
				newSigs.push_back(newSig);
			}
		}
		
		// select point: same as (stk = STK_S)
		newSig = updateS(newPoint, neighbors);
		/*
		cout << "===========" << endl;
		cout << "Sig added :" << endl;
		cout << newSig.toString() << endl;
		cout << "===========" << endl;
		*/
		newSigs.push_back(newSig);
	}
	else if(stk == STK_C){ /// Covered: nothing to do, or select it
		// Case do nothing
		newSig = updateC(newPoint, neighbors);
		newSigs.push_back(newSig);
		// Case select
		newSig = updateS(newPoint, neighbors);
		newSigs.push_back(newSig);
	}
	else{ /// stk == STK_S, Selected: count+1 and set neighbors to covered
		newSig = updateS(newPoint, neighbors);
		newSigs.push_back(newSig);
	}
	
	for(unsigned i=0; i<newSigs.size(); i++){
		newSigs[i].toString();
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
		int index = newSig.getIndex(nb[i]); /// (index = -1) in case of not in sig
		if(index != -1 && newSig.stickers[index] == STK_F){ // a non-covered neighbor
			newSig.stickers[index] = STK_C;
		}
	}
	
	return newSig;
}

Signature Signature::updateS(Point newPoint, vector<vector<vector<Point>>> neighbors){
	Point selected = bag.back();
	Signature newSig = updateF(newPoint, selected, neighbors);
	newSig.select(selected);
	
	return newSig;
}

Signature Signature::updateC(Point newPoint, vector<vector<vector<Point>>> neighbors){
	Signature newSig = updateBagWith(newPoint);
	
	return newSig;
}



vector<Signature> Signature::update(vector<vector<vector<Point>>> neighbors){
	vector<Signature> newSigs;
	Signature newSig;

	// We take the point to be removed next, and its neighbors in the graph.
	Point toCover = bag.back();
	unsigned stk = getSticker(toCover);
	vector<Point> nb = neighbors[toCover.x()][toCover.y()];
	
	if(stk == STK_F){ /// Free (not covered): we need to select this point or a neighbor (so deg+1 new signatures)
		for(unsigned i=0; i<nb.size(); i++){ // select a neighbor
			if(nb[i].isInVector(bag)){
				newSig = updateF(nb[i], neighbors);
				newSigs.push_back(newSig);
			}
		}
		
		// select point: same as (stk = STK_S)
		newSig = updateS(neighbors);
		newSigs.push_back(newSig);
	}
	else if(stk == STK_C){ /// Covered: nothing to do, or select it
		// Case do nothing
		newSig = updateC(neighbors);
		newSigs.push_back(newSig);
		// Case select
		newSig = updateS(neighbors);
		newSigs.push_back(newSig);
	}
	else{ /// stk == STK_S, Selected: count+1 and set neighbors to covered
		newSig = updateS(neighbors);
		newSigs.push_back(newSig);
	}
	
	return newSigs;
}

Signature Signature::updateF(Point selected, vector<vector<vector<Point>>> neighbors){
	Signature newSig = copy();
	newSig.removeLastFromBag();
	
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



Signature Signature::updateS(vector<vector<vector<Point>>> neighbors){
	Point selected = bag.back();
	Signature newSig = updateF(selected, neighbors);
	newSig.select(selected);
	
	return newSig;
}

Signature Signature::updateC(vector<vector<vector<Point>>> neighbors){
	Signature newSig = copy();
	newSig.removeLastFromBag();
	
	return newSig;
}


string Signature::toString(){
	stringstream ss;
	
	ss << "=====" << endl;
	ss << "Sig :" << endl;
	for(unsigned i=0; i<bag.size(); i++){
		ss << bag[i].toString() << " " << stickers[i] << endl;
	}
	ss << "selected :" << endl;
	
	for(unsigned i=0; i<selected.size(); i++){
		ss << selected[i].toString() << endl;
	}
	if(selected.size() == 0){
		ss << "NONE" << endl;
	}
	
	return ss.str();
}








