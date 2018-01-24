
#include <vector>

#include "GraphAux.hpp"
#include "Node.hpp"

using namespace std;

// CONSTRUCTOR
GraphAux::GraphAux(vector<Node> graph){
	this->graph = graph;
}

GraphAux::GraphAux(){}


// the graph MUST have at least one node
Node GraphAux::getNodeDegmin(){
	Node u;
	unsigned degmin = graph.size();
	
	for(unsigned i=0; i<graph.size(); i++){
		unsigned deg = graph[i].getDeg();
		if(deg < degmin){
			u = graph[i];
			degmin = deg;
		}
	}
	
	return u;	
}

// u MUST be in the graph
void GraphAux::removeAndLink(Node u){ 
	vector<Node>::iterator it;

	// remove u
	it = graph.begin();
	while(true){
		if(it->getID() != u.getID()){ // Not u
			it++;
			continue;
		} 
		graph.erase(it);
		break;
	}
	
	// clickify neighbors (and remove u from their neighborhood)
	vector<unsigned> v = u.getNeighborsID();
	for(unsigned i=0; i<v.size(); i++){
		it = graph.begin();
		while(it->getID() != v[i]){ // Look for v[i]
			it++;
		}
		
		it->removeNeighbor(u.getID());
		it->addNeighbors(v);
	}
}
	

bool GraphAux::hasNodes(){
	return (graph.size() > 0);
}


bool GraphAux::isComplete(){
	for(unsigned i=0; i<graph.size(); i++){
		if(graph[i].getDeg() < graph.size()-1){ // Not linked to all other nodes
			return false;
		}
	}
	return true;
}










