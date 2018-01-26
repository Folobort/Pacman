#include <fstream>
#include <iostream>
#include <queue>
#include <string>
#include <sstream>
#include <iterator>
#include <sstream>
#include <vector>
#include <algorithm>

using namespace std;

#include "TreeNode.hpp"
#include "Bag.hpp"
#include "ClassicSignature.hpp"
#include "Node.hpp"

// CONSTRUCTORS
TreeNode::TreeNode(){}

TreeNode::TreeNode(Bag bag){
	this->bag = bag;
	this->parent = new TreeNode;
	isRoot = true;
}
TreeNode::TreeNode(vector<unsigned> bag){
	TreeNode(Bag(bag));
}
	
// GETTERS
bool TreeNode::getIsRoot(){
	return isRoot;
}

Bag TreeNode::getBag(){
	return bag;
}

vector<ClassicSignature> TreeNode::getSigSet(){
	return sigSet;
}
		
	
unsigned TreeNode::getIdToCover(){
	vector<unsigned> parentBg = (parent->getBag()).getContent();
	vector<unsigned> bg = bag.getContent();

	//Normal Case
	for(unsigned j=0; j<bg.size(); j++){
		if(find(parentBg.begin(), parentBg.end(), bg[j])==parentBg.end()){
			return bg[j];
		}
	}
	
	//Case all of vertice are in parent's bag, return error
	unsigned errorBag = 0;
	errorBag--;
	return errorBag;
}
	
// OTHER
void TreeNode::addParent(TreeNode tn){
	*parent = tn;
	isRoot = false;
}

void TreeNode::addChild(TreeNode tn){
	children.push_back(tn);
}
	
bool TreeNode::isGreaterThan(TreeNode tn){
	vector<unsigned> myBagContent = bag.getContent();
	vector<unsigned> otherBagContent = tn.getBag().getContent();
	
	// In a bag, the "center" is the last element (that is, the node that is removed and not present upwards in the TreeDec)
	// Hence, if all elements of tn but the last are in this bag as well, we have a parent link
	
	for(unsigned i=0; i<otherBagContent.size()-1; i++){
		unsigned e = otherBagContent[i];
		vector<unsigned>::iterator it = find(myBagContent.begin(), myBagContent.end(), e);
		
        if(it == myBagContent.end()){ // e is not in myBag, so I am not a parent of tn
			return false;
        }
	}
	
	return true;
}


//Signature
vector<vector<ClassicSignature>> TreeNode::getChildrenSigSet(vector<Node> graph){
	
	vector<vector<ClassicSignature>> childrenSigSet;
	for(unsigned i=0; i<children.size(); i++){
		children[i].computeMySigSet(graph);
		childrenSigSet.push_back(children[i].getSigSet());
	}
	
	return childrenSigSet;
}

void TreeNode::toString(){
	bag.toString();
	if(isRoot){
		cout << " root " << endl;
	}
	else{cout << " no root " << endl;}
	cout << "Parent : " << endl;
	(parent->bag).toString();
	
}






void TreeNode::computeMySigSet(vector<Node> graph){
	if(children.size() == 0){ // Leaf, starting point
		ClassicSignature newSig = ClassicSignature(bag);
		
		unsigned idToCover = getIdToCover();
		sigSet = newSig.update(idToCover, graph);
		
		return;
	}
	
	// = Case NOT leaf =
	
	// Gather children signatures
	vector<vector<ClassicSignature>> childrenSigSets = getChildrenSigSet(graph);
	unsigned n = childrenSigSets.size();
	
	vector<unsigned> iVect = vector<unsigned> (n, 0);
	
	while(iVect[0] < childrenSigSets[0].size()){
		// select a combination
		vector<ClassicSignature> chosenSigs;
		for(unsigned j=0; j<childrenSigSets.size(); j++){
			chosenSigs.push_back(childrenSigSets[j][iVect[j]]);
		}
		
		ClassicSignature mySig = ClassicSignature(bag);
		chosenSigs.push_back(mySig);
		
		// update iVect for later
		iVect[n-1]++;
		
		unsigned k=n-1;
		while(k > 0){
			if(iVect[k] == childrenSigSets[k].size()){ // index to large => "reset"
				iVect[k] = 0;
				iVect[k-1]++;
				k--;
			} else{
				break;	// valid iVect
			}
		}
		
		// Now compute sum
		ClassicSignature sumSig = sumSignature(chosenSigs, graph);
		// And compute all signatures deduced from it
		unsigned idToCover = getIdToCover();
		
		vector<ClassicSignature> newSigs = sumSig.update(idToCover, graph);
		
		
		// Finally, store them in my sigSet
		for(unsigned l=0; l<newSigs.size(); l++){
			sigSet.push_back(newSigs[l]);
		}
	}
	
	// Clean duplicates
	CleanSigSet();
}



ClassicSignature TreeNode::sumSignature(vector<ClassicSignature> chosenSigs, vector<Node> graph){
	ClassicSignature sumSig;
	Bag sumBag;

	// Create bag with all ids (and create the signature)
	for(unsigned i=0; i<chosenSigs.size(); i++){
		vector<unsigned> ids = chosenSigs[i].getBag().getContent();
		
		for(unsigned j=0; j<ids.size(); j++){
			sumBag.add(ids[j]);
		}
	}
	
	sumSig = ClassicSignature(sumBag);
	
	
	// Prepare the sticker vector to all STK_F
	vector<unsigned> bg = sumBag.getContent();
	vector<unsigned> sumStk = vector<unsigned> (bg.size(), STK_F);
	
	// sum the stickers
	for(unsigned index=0; index<bg.size(); index++){
		for(unsigned i=0; i<chosenSigs.size(); i++){
			vector<unsigned> ids = chosenSigs[i].getBag().getContent();
			vector<unsigned> stk = chosenSigs[i].getStickers();
	
			for(unsigned j=0; j<ids.size(); j++){
				if(bg[index] == ids[j]){ // this id is also in the chosenBag
					if(stk[j] == STK_S){
						sumStk[index] = STK_S;
					}
					else if(stk[j] == STK_C && sumStk[index] == STK_F){
						sumStk[index] = STK_C;
					}
				}
			}
		}
	}
	
	// Make sure dominance is right
	for(unsigned index=0; index<bg.size(); index++){
		Node u = getNodeWithID(bg[index], graph);
		
		if(sumStk[index] == STK_S){
			vector<unsigned> v = u.getNeighborsID();
			
			for(unsigned index2=0; index2 < bg.size(); index2++){
				if(find(v.begin(), v.end(), bg[index2])!=v.end()){ // if another element of the bag is its neighbor
					if(sumStk[index2] == STK_F){	// and not covered yet
						sumStk[index2] = STK_C;
					}
				}
			}
		}
	}
	
	//Set Stickers in SumSig
	for(unsigned index=0; index<bg.size(); index++){
		sumSig.setSticker(bg[index], sumStk[index]);
	}
	
	// Sum the selected vectors (no risk of adding the same id twice in the selection)
	for(unsigned i=0; i<chosenSigs.size(); i++){
		vector<unsigned> sld = chosenSigs[i].getSelectedID();
		for(unsigned j=0; j<sld.size(); j++){
			sumSig.select(sld[j]);
		}
	}
	
	return sumSig;
}




Node TreeNode::getNodeWithID(unsigned id, vector<Node> graph){
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





void TreeNode::CleanSigSet(){
	vector<ClassicSignature> cleanedSigs;
	
	vector<bool> keepIndex(sigSet.size(), true);
	
	for(unsigned i=0; i<sigSet.size(); i++){
		if(keepIndex[i]){
			for(unsigned j=i+1; j<sigSet.size(); j++){
				if(keepIndex[j] && sigSet[i].equals(sigSet[j])){ // same apparent signature
					if(sigSet[i].getSelectedID().size() <= sigSet[j].getSelectedID().size()){
						keepIndex[j] = false;
					}else{
						keepIndex[i] = false;
					}
				}
			}
		}
	}
	
	for(unsigned i=0; i<sigSet.size(); i++){
		if(keepIndex[i]){
			cleanedSigs.push_back(sigSet[i]);
		}
	}
	
	sigSet = cleanedSigs;	
}














