#ifndef GRAPHAUX_H_
#define GRAPHAUX_H_

#include <vector>

#include "Node.hpp"

using namespace std;

class GraphAux{
	// == DATA ==
	vector<Node> graph;
	
	// == PROTOTYPES ==
	public:
	
	// CONSTRUCTORS
	GraphAux(vector<Node> graph);
	GraphAux();
	
	// GETTERS
	
	
	// OTHER
	Node getNodeDegmin();
	void removeAndLink(Node u);
	
	bool hasNodes();
	bool isComplete();
};


#endif

