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

// CONSTRUCTORS
TreeNode::TreeNode(Bag bag){
	this->bag = bag;
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

vector<ClassicSignature> getSigSet(){
	return sigSet;
}
	
unsigned getToCover(){
	vector<unsigned> parentBag = *parent.getBag();
	
	//Normal Case
	for(unsigned j=0; j<bag.size(); j++){
		if(find(parentBag.begin(), parentBag.end(), bag[j])==parentBag.end()){
			return bag[j];
		}
	}
	
	//Case all of vertice are in parent's bag, return error
	unsigned errorBag = 0;
	errorBag--;
	return errorBag;
}
	
	
// OTHER
void TreeNode::addParent(TreeNode tn){
	parent = &tn;
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


void TreeNode::toString(){
	bag.toString();
	if(isRoot){
		cout << " root " << endl;
	}
	else{cout << " no root " << endl;}
}



