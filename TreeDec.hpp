#ifndef TREEDEC_H_
#define TREEDEC_H_

#include <vector>

using namespace std;

#include "TreeNode.hpp"
#include "Bag.hpp"

// (Actually a forest structure, but ends up as a tree)
class TreeDec{
	// == DATA ==
	vector<TreeNode> roots;
	bool isEmpty;	// Useful to add the very first element
	
	// == PROTOTYPES ==
	public:
	
	// CONSTRUCTORS
	TreeDec();
	TreeDec(TreeNode root);
	
	// OTHER
	void addBag(Bag bag);
	void addBag(vector<unsigned> bag);
};


#endif

