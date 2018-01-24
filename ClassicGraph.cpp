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
/// TO DO!!!
ClassicGraph::ClassicGraph(vector<Node> graph){
	this->graph = graph;
	dist = vector<unsigned> (graph.size(), 0);
}

/* OLD ctor (24/01 morning)
ClassicGraph::ClassicGraph(vector<vector<unsigned>> adj){
    this->adjacency=adj;
}
*/


//============== Tree decomposition==================================
//Creates a rooted tree decomposition

/// OK, TO COMPILE
void ClassicGraph::treeDecomposition(){
	GraphAux gAux = GraphAux(graph); /// Copy of the node structure ("newAdj")
		
	while(gAux.hasNodes()){ /// "adjList not empty"
		// Case gAux is complete
		if(gAux.isComplete()){
			Bag bag = Bag(graph[0].getNeighborsID());
			bag.add(graph[0].getID());
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

/* OLD TreeDecomposition & co (24/01 morning)
void ClassicGraph::treeDecomposition(){
    vector<vector<unsigned>> adjList = adjacency;
    vector<vector<unsigned>> newadjList;
    vector<unsigned> listVertice = listOfVertice(adjList);
    vector<unsigned> bag;
    unsigned degMinIndex;
    do{
        if(isComplete(adjList)){
            bag=listVertice;
            adjList.clear();
        }
        else{
            degMinIndex=verticeOfDegreeMin(adjList);
            bag=adjList[degMinIndex];
            bag.push_back(listVertice[degMinIndex]);
            newadjList=updateAdjacency(adjList, listVertice, degMinIndex);
            listVertice.erase(listVertice.begin()+degMinIndex-1);
            treeRooting(bag);
        }
        tree.push_back(bag);
    }while(!adjList.empty());
}


//Find son in tree
//If new bag is father's of an already existing bag then :
//1) Bag as only one vertice different, with bag constructions rules it must be the last one;
//2) New bag as then all neighbors of this vertice;

void ClassicGraph::treeRooting(vector<unsigned> newBag){
    vector<unsigned> bag;
    bool isSon = false;
    for(unsigned i=0; i<tree.size(); i++){
        bag=tree[i];
        isSon=true;
        for(unsigned j=0; j<bag.size()-1; j++){
            if(find(newBag.begin(), newBag.end(), bag[j])==newBag.end()){
                isSon=false;
            }
        }
        if(isSon){
            father.push_back(i);
            sons[i].push_back(tree.size());
        }
    }
}

//Erase u's line in adjacency list and add to each u's neighbors the new neighbors.
vector<vector<unsigned>> ClassicGraph::updateAdjacency(vector<vector<unsigned>> adjacencyList, vector<unsigned> verticeList, unsigned verticeIndex){
    vector<vector<unsigned>> adjList = adjacencyList;
    vector<unsigned> neighbors = adjList[verticeIndex];
    adjList.erase(adjList.begin()+verticeIndex-1);
    unsigned u=verticeList[verticeIndex];
    unsigned v;
    unsigned index;
    vector<unsigned> neighborsToAdd;
    if(neighbors.size()>0){
        for(unsigned i=0; i<neighbors.size(); i++){
            v=neighbors[i];
            neighborsToAdd=neighbors;
            neighborsToAdd.erase(neighbors.begin()+i-1);
            index=distance(verticeList.begin(), find(verticeList.begin(), verticeList.end(), v));
            adjList[index].erase(find(adjList[index].begin(), adjList[index].end(), u));
            addNewNeighbors(adjList[index],neighborsToAdd);
        }
    }
    return adjList;
}

vector<unsigned> ClassicGraph::addNewNeighbors(vector<unsigned> neighbors, vector<unsigned> neighborsToAdd){
    vector<unsigned> newNeighbors = neighbors;
    for(unsigned i=0; i<neighborsToAdd.size(); i++){
        if(find(neighbors.begin(), neighbors.end(), neighborsToAdd[i])==neighbors.end()){
            newNeighbors.push_back(neighborsToAdd[i]);
        }
    }
    return newNeighbors;
}
*/




//===================Brute force===========================================================
/* OLD 24/01 morning
vector<unsigned> ClassicGraph::k_dominant(unsigned k, vector<unsigned> S){
	if(S.size() == k){
		if(isDominating(S)){
			return S;
		}
		vector<unsigned> emptyV;
		return emptyV; /// Empty vector on failure
	}

	unsigned v = undominatedDegreeMin(S);

	vector<unsigned> neighbors = adjacency[v];
	for(vector<unsigned>::iterator i=neighbors.begin(); i!=neighbors.end(); ){
        if(isDominated(S, *i)){
            neighbors.erase(i);
        }
        else{i++;}
	}

	for(unsigned i=0; i<neighbors.size(); i++){
		vector<unsigned> S2 = S;
		S2.push_back(neighbors[i]);

		if(k_dominant(k, S2).size() != 0){
			return S2;
		}
	}

	vector<unsigned> emptyV;
	return emptyV; /// All sub-S failed
}
* */

/// OK, TO COMPILE
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

		if(k_dominant(k, S2).size() != 0){ // Not a fail, so bring it back up
			return S2;
		}
	}

	vector<unsigned> emptyV;
	return emptyV; // All sub-S failed
}

/* OLD 24/01 morning
unsigned ClassicGraph::undominatedDegreeMin(vector<unsigned> S){
	vector<vector<unsigned>> adjList = adjacency;
	unsigned degMin = adjList[0].size();
    unsigned degMinVertice = 0;
    for(unsigned i=1; i<adjList.size(); i++){
        if(adjList[i].size()<degMin && !isDominated(S,i)){
            degMin=adjList[i].size();
            degMinVertice=i;
        }
    }
    if(degMinVertice==0 && isDominated(S,0)){
        return -1;
    }
    else{return degMinVertice;}
}


unsigned ClassicGraph::undominatedDegreeMin(vector<unsigned> S){
	vector<vector<unsigned>> adjList = adjacency;
	unsigned degMin = adjList[0].size();
    unsigned degMinVertice = 0;
    for(unsigned i=1; i<adjList.size(); i++){
        if(adjList[i].size()<degMin && !isDominated(S,i)){
            degMin=adjList[i].size();
            degMinVertice=i;
        }
    }
    if(degMinVertice==0 && isDominated(S,0)){
        return -1;
    }
    else{return degMinVertice;}
}

bool ClassicGraph::isDominating(vector<unsigned> S){
    for(unsigned x=0; x<adjacency.size(); x++){
        if(!isDominated(S, x)){
            return false;
        }
    }
	return true;
}

bool ClassicGraph::isDominated(vector<unsigned> S, unsigned vertice){
	vector<unsigned> neighbors = adjacency[vertice];
	bool dominated = false;
	if(find(S.begin(), S.end(), vertice)!=S.end()){
        dominated = true;
	}
	else{
        for(unsigned x=0; x<neighbors.size(); x++){
            if(find(S.begin(), S.end(), neighbors[x])!=S.end()){
                    dominated=true;
                    break;
                }
            }
	}
	return dominated;
}
* */

/// OK, TO COMPILE
unsigned ClassicGraph::undominatedDegreeMin(vector<unsigned> S){
	Node u = graph[0];
	unsigned degmin = graph.size();
	
	for(unsigned i=0; i<graph.size(); i++){
		unsigned deg = graph[i].getDeg();
		if(deg < degmin && !isDominated(S, u)){
			u = graph[i];
			degmin = deg;
		}
	}
	
	// Case all nodes are dominated
	if(isDominated(S, u)){ 
		unsigned errorCode = 0;
		errorCode--;
		return errorCode; // Returns MAX_INT as an error
	}
	
	// Normal case
	return u.getID();
}

/// OK, TO COMPILE
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

/// OK, TO COMPILE
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

/// OK, TO COMPILE
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

/// OK, TO COMPILE
bool ClassicGraph::isDominating(vector<unsigned> S){
	for(unsigned i=0; i<graph.size(); i++){
        if(!isDominated(S, graph[i])){
            return false;
        }
    }
	return true;
}

//==================================  Dijkstra  ============================================================

/* OLD 24/01 morning
unsigned ClassicGraph::Dijkstra(vector<unsigned> S){
	processed = vector<bool> (adjacency.size(),false);
	vector<unsigned> toProcessNow;
	vector<unsigned> neighbors;

	for(unsigned i=0; i<S.size(); i++){ /// Set tuple elements to dist 0
		dist[S[i]] = 0;
		processed[S[i]]= true;
		toProcessNow.push_back(tuple[i]);
	}

	unsigned maxDist = 0;
	while(toProcessNow.size() > 0){
		unsigned point = toProcessNow.front();
		neighbors=adjacency[point];
		toProcessNow.erase(toProcessNow.begin()); // pop_front()

		if(dist[point] > maxDist){
			maxDist = dist[point];
		}

		for(unsigned j=0; j<neighbors.size(); j++){ /// Process neighbors
			if(!processed[neighbors[j]]){
                processed[neighbors[j]] = true;
                dist[neighbors[j]] = dist[point] + 1;
                toProcessNow.push_back(neighbors[j]);
			}
		}
	}

	return maxDist;
}
*/

/// OK, TO COMPILE
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

/// OK, TO COMPILE
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

/*
//Enumerate Kuplet - OK
vector<unsigned> nextKuplet(vector<unsigned> S){
    for(unsigned k=1; k<S.size(); k++){
        if((S[k]-S[k-1])!=1){
            unsigned newVertice=S[k]+1;
            S[k]=newVertice;
            return S;
        }
    }
    unsigned newVertice=S[S.size()-1]+1;
    S[S.size()-1]=newVertice;
    return S;
}
*/






/* OLD 24/01 morning
vector<unsigned> ClassicGraph::listOfVertice(vector<vector<unsigned>> adjacencyList){
    vector<unsigned> listVertice;
    for(unsigned i=0; i<adjacencyList.size(); i++){
            listVertice.push_back(i);
    }
    return listVertice;
}
*/


/* OLD 24/01 morning
unsigned ClassicGraph::verticeOfDegreeMin(vector<vector<unsigned>> adjacencyList){
    unsigned degMin = adjacencyList[0].size();
    unsigned degMinVertice = 0;
    for(unsigned i=1; i<adjacencyList.size(); i++){
        if(adjacencyList[i].size()<degMin){
            degMin=adjacencyList[i].size();
            degMinVertice=i;
        }
    }
    return degMinVertice;
}


//Check if left vertices form a complete graph for tree decomposition
bool ClassicGraph::isComplete(vector<vector<unsigned>> adjacencyList){
    for(unsigned i=0; i<adjacencyList.size(); i++){
            if(adjacencyList[i].size()<adjacencyList.size()-1){
                return false;
            }
    }
    return true;
}
*/





