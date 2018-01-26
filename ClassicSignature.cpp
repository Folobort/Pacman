#include <fstream>
#include <iostream>
#include <queue>
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <algorithm>

#include "ClassicSignature.hpp"
#include "Node.hpp"
#include "TreeDec.hpp"
#include "TreeNode.hpp"
#include "Signature.hpp"
#include "GraphAux.hpp"


using namespace std;

// CONSTRUCTORS
ClassicSignature::ClassicSignature(){}

ClassicSignature::ClassicSignature(Bag bg){
	treeBag = bg;
	stickers = vector<unsigned>(bg.getContent().size(),STK_F);
}

ClassicSignature ClassicSignature::copy(){
	ClassicSignature newSig;
	
	newSig.treeBag = treeBag;
	newSig.stickers = stickers;
	newSig.selectedID = selectedID;
	
	return newSig;
}

// GETTERS
int ClassicSignature::getIndex(unsigned id){
	vector<unsigned> bag = treeBag.getContent();
	for(unsigned i=0; i<bag.size(); i++){
		if(bag[i] == id){
			return i;
		}
	}
	// Not in Signature List
	return -1;
}

unsigned ClassicSignature::getSticker(unsigned id){
	int index = getIndex(id);
	if(index != -1){
		return stickers[index];
	} else{
		cout << "SIGNATURE::GETSTICKER" << endl;
		return 23;
	}
}

Bag ClassicSignature::getBag(){
	return treeBag;
}

vector<unsigned> ClassicSignature::getStickers(){
	return stickers;
}

vector<unsigned> ClassicSignature::getSelectedID(){
	return selectedID;
}





Node ClassicSignature::getNodeWithID(unsigned id, vector<Node> graph){
	/// This might quite slow if the graph is big
	/// => If we have time left, try mapping the nodes?
	vector<Node>::iterator it = graph.begin();
	
	while(true){
		if(it->getID() == id){
			return (*it);
		}
		it++;
	}
}

// SETTERS
void ClassicSignature::setSticker(unsigned id, unsigned stk){
	int index = getIndex(id);
	
	if(index != -1){
		stickers[index] = stk;
	}/// In fact, the else case happens and does not matter
}

void ClassicSignature::select(unsigned idSelected){
	selectedID.push_back(idSelected);
}


// COMPARATORS
bool ClassicSignature::equals(ClassicSignature sig){ // only with same bags (ie, same node of the tree dec)
	return equal(stickers.begin(), stickers.end(), sig.getStickers().begin());
}

bool ClassicSignature::isBetterFinalSig(ClassicSignature sig){
	return (getSelectedID().size() <= sig.getSelectedID().size());
}


// OTHERS

void ClassicSignature::remove(unsigned id){
	int index = getIndex(id);
	
	if(index != -1){
		treeBag.remove(id);
		stickers.erase(stickers.begin()+index);
	}
}


vector<ClassicSignature> ClassicSignature::update(unsigned idToCover, vector<Node> graph){
	vector<ClassicSignature> newSigs;
	ClassicSignature newSig;

	// We take the point to be removed next, and its neighbors in the graph.
	Node u = getNodeWithID(idToCover, graph);
	
	unsigned stk = getSticker(idToCover);
	
	vector<unsigned> nb = u.getNeighborsID();
	
	if(stk == STK_F){ /// Free (not covered): we need to select this point or a neighbor (so deg+1 new signatures)
		for(unsigned i=0; i<nb.size(); i++){ // select a neighbor
			if(isInBag(nb[i])){
				newSig = updateF(idToCover, nb[i], graph);
				newSigs.push_back(newSig);
			}
		}
		
		// select point: same as (stk = STK_S)
		newSig = updateS(idToCover, graph);
		newSigs.push_back(newSig);
	}
	else if(stk == STK_C){ /// Covered: nothing to do, or select it
		// Case do nothing
		newSig = updateC(idToCover);
		newSigs.push_back(newSig);
		// Case select
		newSig = updateS(idToCover, graph);
		newSigs.push_back(newSig);
	}
	else{ /// stk == STK_S, Selected: count+1 and set neighbors to covered
		newSig = updateS(idToCover, graph);
		newSigs.push_back(newSig);
	}
	
	return newSigs;
}


ClassicSignature ClassicSignature::updateF(unsigned idToCover, unsigned idSelected, vector<Node> graph){
	ClassicSignature newSig = copy();
	newSig.remove(idToCover);
	
	// set the point to selected
	newSig.setSticker(idSelected, STK_S);
	
	
	// each neighbor which is not covered (STK_F) becomes covered (STK_C)
	Node v = getNodeWithID(idSelected, graph);
	vector<unsigned> nb = v.getNeighborsID();
	
	for(unsigned i=0; i<nb.size(); i++){
		int index = newSig.getIndex(nb[i]); /// (index = -1) if not in bag
		if(index != -1 && newSig.stickers[index] == STK_F){ // a non-covered neighbor in the bag
			newSig.stickers[index] = STK_C;
		}
	}
	
	return newSig;
}

ClassicSignature ClassicSignature::updateS(unsigned idSelected, vector<Node> graph){
	
	ClassicSignature newSig = updateF(idSelected, idSelected, graph);
	newSig.select(idSelected);
	
	return newSig;
}

ClassicSignature ClassicSignature::updateC(unsigned idCovered){
	ClassicSignature newSig = copy();
	newSig.remove(idCovered);
	
	return newSig;
}


bool ClassicSignature::isInBag(unsigned id){
	vector<unsigned> bag = treeBag.getContent();
	return (find(bag.begin(), bag.end(), id)!=bag.end());
}


string ClassicSignature::toString(){
	vector<unsigned> bg = treeBag.getContent();
	
	stringstream ss;
	
	ss << "=====" << endl;
	ss << "Sig :" << endl;
	for(unsigned i=0; i<bg.size(); i++){
		ss << bg[i] << " " << stickers[i] << endl;
	}
	ss << "selected :" << endl;
	
	for(unsigned i=0; i<selectedID.size(); i++){
		ss << selectedID[i] << endl;
	}
	if(selectedID.size() == 0){
		ss << "NONE" << endl;
	}
	
	return ss.str();
}








