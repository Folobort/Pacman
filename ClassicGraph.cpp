#include <fstream>
#include <iostream>
#include <queue>
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <algorithm>

#include "ClassicGraph.hpp"
#include "ClassicKuplet.hpp"
#include "Graph.hpp"
#include "Parser.hpp"
#include "Signature.hpp"

using namespace std;

// CONSTRUCTORS
ClassicGraph::ClassicGraph(vector<Node> graph){
	this->graph = graph;
	dist = vector<unsigned> (graph.size(), 0);
}

//============== Tree decomposition==================================
//Creates a rooted tree decomposition

void ClassicGraph::treeDecomposition(){
	GraphAux gAux = GraphAux(graph);
		
	while(gAux.hasNodes()){
		// Case gAux is complete
		if(gAux.isComplete()){
			Node u = gAux.getFirstNode();
			
			Bag bag = Bag(u.getNeighborsID());
			bag.add(u.getID());
			
			treeDec.addBag(bag);
			
			gAux = GraphAux(); // Quickly empties the graph
			
			continue;
		}
		
		// Normal case
		
		// Get one of the best nodes
		Node u = gAux.getNodeDegmin();
		// Add new bag to the tree
		Bag bag = Bag(u.getNeighborsID());
		bag.add(u.getID());
		
		treeDec.addBag(bag);
		
		// Remove the node and clickify the neighbors
		gAux.removeAndLink(u);		
	}
}

void ClassicGraph::solveTree(){
	bestSelectedSet = treeDec.computeBestSelected(graph);
}


//===================Brute force===========================================================
vector<unsigned> ClassicGraph::k_dominant(unsigned k, vector<unsigned> S){
	
	// Case size == k
	if(S.size() == k){
		if(isDominating(S)){
			return S;
		}
		vector<unsigned> emptyV;
		return emptyV; /// Empty vector on failure
	}

	// Case size < k
	unsigned v = undominatedDegreeMin(S);
	
	unsigned errorCode = 0;
	errorCode--;
	if(v == errorCode){ // All nodes are dominated by a set smaller than k
		return S;
		
	}
	
	// Normal case
	vector<unsigned> undominatedNeighbors = getNodeWithID(v).getNeighborsID();
	undominatedNeighbors.push_back(v);
	
	// clean set of actually dominated nodes
	vector<unsigned>::iterator it = undominatedNeighbors.end()-1;
	while(it != undominatedNeighbors.begin()){
		if(isDominated(S, *it)){
			undominatedNeighbors.erase(it);
		}
		it--;
	}
	if(isDominated(S, *it)){
		undominatedNeighbors.erase(it);
	}

	for(unsigned i=0; i<undominatedNeighbors.size(); i++){
		vector<unsigned> S2 = S;
		S2.push_back(undominatedNeighbors[i]);
		
		vector<unsigned> S3 = k_dominant(k, S2);
		
		
		if(S3.size() != 0){ // Not a fail, so bring it back up
			return S3;
		}
	}

	vector<unsigned> emptyV;
	return emptyV; // All sub-S failed
}

unsigned ClassicGraph::undominatedDegreeMin(vector<unsigned> S){
	
	Node u = graph[0];
	unsigned degmin = graph.size();
	for(unsigned i=0; i<graph.size(); i++){
		unsigned deg = graph[i].getDeg();
		if(deg < degmin && !isDominated(S, graph[i])){
			u = graph[i];
			degmin = deg;
		}
	}
	cout << endl;
	// Case all nodes are dominated
	if(degmin == graph.size()){ 
		unsigned errorCode = 0;
		errorCode--;
		return errorCode; // Returns MAX_INT as an error
	}
	
	// Normal case
	return u.getID();
}

Node ClassicGraph::getNodeWithID(unsigned id){
	/// This might quite slow if the graph is big
	/// => If we have time left, try mapping the nodes?
	vector<Node>::iterator it = graph.begin();
	
	while(true){
		if(it->getID() == id){
			return (*it);
		}
		it++;
	}
}

bool ClassicGraph::isDominated(vector<unsigned> S, unsigned id){
	vector<unsigned>::iterator it;
	
	// Case node already in S
	it = find(S.begin(), S.end(), id);
	if(it != S.end()){
		return true;
	}
	
	// Normal case
	Node u = getNodeWithID(id);
    return isDominated(S, u);
}

bool ClassicGraph::isDominated(vector<unsigned> S, Node u){
	vector<unsigned>::iterator it;
	// Case node already in S
	it = find(S.begin(), S.end(), u.getID());
	if(it != S.end()){
		return true;
	}
	
	// Normal case
    vector<unsigned> v = u.getNeighborsID();
    
	for(unsigned i=0; i<v.size(); i++){
		it = find(S.begin(), S.end(), v[i]);
		if(it != S.end()){ // one neighbor is in S
			return true;
        }
    }
	return false;
}

bool ClassicGraph::isDominating(vector<unsigned> S){
	for(unsigned i=0; i<graph.size(); i++){
        if(!isDominated(S, graph[i])){
            return false;
        }
    }
	return true;
}

//==================================  Dijkstra  ============================================================

unsigned ClassicGraph::Dijkstra(vector<unsigned> S){
	vector<bool> processed = vector<bool> (graph.size(),false);
	vector<unsigned> toProcessNow;

	for(unsigned i=0; i<S.size(); i++){ /// Set tuple elements to dist 0
		dist[S[i]] = 0;
		processed[S[i]]= true;
		toProcessNow.push_back(S[i]);
	}

	unsigned maxDist = 0;
	while(toProcessNow.size() > 0){
		Node u = getNodeWithID(toProcessNow.front());
		vector<unsigned> v = u.getNeighborsID();
		toProcessNow.erase(toProcessNow.begin()); // pop_front()

		if(dist[u.getID()] > maxDist){
			maxDist = dist[u.getID()];
		}

		for(unsigned j=0; j<v.size(); j++){ /// Process neighbors
			if(!processed[v[j]]){
                processed[v[j]] = true;
                dist[v[j]] = dist[u.getID()] + 1;
                toProcessNow.push_back(v[j]);
			}
		}
	}

	return maxDist;
}

vector<unsigned> ClassicGraph::bestDijkstra(unsigned k){
	ClassicKuplet ck = ClassicKuplet(graph.size());
	
	//Creates first set
	vector<unsigned> S = ck.firstElement(k);
	vector<unsigned> bestSet = S;
	unsigned bestDist = Dijkstra(S);;

	while(S[0] != (graph.size()-1)-(k-1)){ // Format of the last set
		S = ck.nextElement(S);
		unsigned otherDist = Dijkstra(S);
		
		if(otherDist < bestDist){
			bestDist = otherDist;
			bestSet = S;
		}
	}
	
	return bestSet;
}

string ClassicGraph::toString(){
	stringstream ss;
	
	ss << "graph G {" << endl;
	
	// Declare nodes
	for(unsigned i=0; i<graph.size(); i++){
		string shapeStr;
		if(find(bestSelectedSet.begin(), bestSelectedSet.end(), graph[i].getID()) != bestSelectedSet.end()){
			shapeStr = "shape = box";
		} else{
			shapeStr = "shape = circle";
		}
		
		ss << graph[i].getID() << "[" << shapeStr << "];" << endl;
	}
	
	// Edges
	for(unsigned i=0; i<graph.size(); i++){
		Node u = graph[i];
		vector<unsigned> v = u.getNeighborsID();
		for(unsigned j=0; j<u.getDeg(); j++){
			if(u.getID() < v[j]){
				ss << u.getID() << "--" << v[j] << ";" << endl;
			}
		}
	}
	

	ss << "}";
	
	return ss.str();
}












