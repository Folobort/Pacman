#include <fstream>
#include <iostream>
#include <queue>
#include <string>
#include <sstream>
#include <map>
#include <algorithm>
#include <iterator>
#include <vector>

using namespace std;

#include "TreeDec.hpp"
#include "TreeNode.hpp"
#include "Bag.hpp"


// CONSTRUCTORS

TreeDec::TreeDec(){
	isEmpty = true;
}

TreeDec::TreeDec(TreeNode root){
	roots.push_back(root);
	isEmpty = false;
}


// OTHER

void TreeDec::addBag(Bag bag){
	TreeNode tn = TreeNode(bag);
	
	// A) Case empty: the first bag is a root
	if(isEmpty){
		roots.push_back(tn);
		isEmpty = false;
		return;
	}
	
	// B) Case not empty:
	// If the bag is "greater" than a root or several roots, then the bag becomes a parent of these
	for(vector<TreeNode>::iterator it = roots.begin(); it != roots.end(); it++){
		if(tn.isGreaterThan(*it)){
			tn.addChild(*it);
			it->addParent(tn);
		}
	}
	// clean the root list (starting at the end to keep track of iterator when erasing)
	vector<TreeNode>::iterator it = roots.end()-1;
	while(it != roots.begin()){
		if(it->getIsRoot() == false){ // no longer a root
			roots.erase(it);
		}
		it--;
	}
	if(it->getIsRoot() == false){ // no longer a root
		roots.erase(it);
	}
	
	// In any case, this new bag becomes a root (linking trees, or a new tree if no child found)
	roots.push_back(tn);
}

void TreeDec::addBag(vector<unsigned> bag){
	addBag(Bag(bag));
}
	
void TreeDec::toString(){
	for(unsigned i=0; i<roots.size(); i++){
		roots[i].toString();
	}
}	

vector<unsigned> TreeDec::computeBestSelected(vector<Node> graph){
	// Only compute the first root sigSet (this function is usually called when there is only one)
	roots[0].computeMySigSet(graph);
	
	// get root SigSet
	vector<ClassicSignature> sigSet = roots[0].getSigSet();
	unsigned n = sigSet[0].getBag().getContent().size(); // size of the last bag
	
	// reduce last bag
	for(unsigned i=0; i<n; i++){
		unsigned id = sigSet[0].getBag().getContent()[0];
		vector<ClassicSignature> newSigSet;
		
		for(unsigned j=0; j<sigSet.size(); j++){
			vector<ClassicSignature> upd = sigSet[j].update(id, graph);
			
			for(unsigned k=0; k<upd.size(); k++){
				newSigSet.push_back(upd[k]);
			}
		}
		
		sigSet = cleanSigSet(newSigSet);
	}
	
	return sigSet[0].getSelectedID(); // In case of equally best sets, return only one selected set
}





vector<ClassicSignature> TreeDec::cleanSigSet(vector<ClassicSignature> sigSetToClean){
	vector<ClassicSignature> cleanedSigs;
	
	vector<bool> keepIndex(sigSetToClean.size(), true);
	
	for(unsigned i=0; i<sigSetToClean.size(); i++){
		if(keepIndex[i]){
			for(unsigned j=i+1; j<sigSetToClean.size(); j++){
				if(keepIndex[j] && sigSetToClean[i].equals(sigSetToClean[j])){ // same apparent signature
					if(sigSetToClean[i].getSelectedID().size() <= sigSetToClean[j].getSelectedID().size()){
						keepIndex[j] = false;
					}else{
						keepIndex[i] = false;
					}
				}
			}
		}
	}
	
	for(unsigned i=0; i<sigSetToClean.size(); i++){
		if(keepIndex[i]){
			cleanedSigs.push_back(sigSetToClean[i]);
		}
	}
	
	return cleanedSigs;	
}

















