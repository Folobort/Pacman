#ifndef TREENODE_H_
#define TREENODE_H_

#include <vector>

using namespace std;

#include "Bag.hpp"

class TreeNode{
	// == DATA ==
	Bag bag;
	
	bool isRoot;
	TreeNode* parent;
	vector<TreeNode> children;
	
	// == PROTOTYPES ==
	public:
	
	// CONSTRUCTORS
	TreeNode(Bag bag);
	TreeNode(vector<unsigned> bag);
	
	// GETTERS
	bool getIsRoot();
	Bag getBag();
	
	// OTHER
	void addParent(TreeNode tn);
	void addChild(TreeNode tn);
	
	bool isGreaterThan(TreeNode tn);
};


#endif

