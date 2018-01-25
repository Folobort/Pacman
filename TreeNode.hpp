#ifndef TREENODE_H_
#define TREENODE_H_

#include <vector>

using namespace std;

#include "Bag.hpp"
#include "ClassicSignature.hpp"
#include "Node.hpp"

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
	
	unsigned getIdToCover();
	
	Node getNodeWithID(unsigned id, vector<Node> graph);
	
	// OTHER
	void addParent(TreeNode tn);
	void addChild(TreeNode tn);
	
	bool isGreaterThan(TreeNode tn);
	
	void toString();
	
	
	void computeMySigSet(vector<Node> graph);
	vector<vector<ClassicSignature>> getChildrenSigSet(vector<Node> graph); //Update sigSet with children's
	
	ClassicSignature sumSignature(vector<ClassicSignature> chosenSigs, vector<Node> graph);


};


#endif

