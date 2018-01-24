
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
	



