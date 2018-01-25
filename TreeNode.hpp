#ifndef TREENODE_H_
#define TREENODE_H_

#include <vector>

using namespace std;

#include "Bag.hpp"
#include "ClassicSignature.hpp"

class TreeNode{
	// == DATA ==
	Bag bag;
	
	bool isRoot;
	TreeNode* parent;
	vector<TreeNode> children;
	
	vector<ClassicSignature> sigSet;
	
	// == PROTOTYPES ==
	public:
	
	// CONSTRUCTORS
	TreeNode(Bag bag);
	TreeNode(vector<unsigned> bag);
	
	// GETTERS
	bool getIsRoot();
	Bag getBag();
	
	vector<ClassicSignature> getSigSet();
	
	unsigned getToCover();
	
	// OTHER
	void addParent(TreeNode tn);
	void addChild(TreeNode tn);
	
	bool isGreaterThan(TreeNode tn);
	
	void toString();
	
	
	void computeMySigSet(vector<vector<ClassicSignature>> childrenSigSet);
	vector<vector<ClassicSignature>> getChildrenSigSet(); //Update sigSet with children's
	
	
};


#endif

