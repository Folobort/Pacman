#ifndef NODE_H_
#define NODE_H_

#include <vector>

using namespace std;

/// This class represents a node,
/// simply by its ID and to which other node it is linked to
class Node{
	// == DATA ==
	unsigned id;
	vector<unsigned> neighborsID;
	
	// == PROTOTYPES ==
	public:
	
	// CONSTRUCTORS
	Node(); /// Should not be used as it is (only to "prepare" a variable name!)
	Node(unsigned id, vector<unsigned> neighborsID);
	
	// GETTERS
	unsigned getID();
	vector<unsigned> getNeighborsID();
	unsigned getDeg();
	
	// OTHER
	void addNeighbor(unsigned v);			// Add if not already present (and not itself)
	void addNeighbors(vector<unsigned> v);
	void removeNeighbor(unsigned v);		// Remove if present
};


#endif
