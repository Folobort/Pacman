#include <fstream>
#include <iostream>
#include <queue>
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <algorithm>

#include "ClassicGraph.h"
#include "Graph.hpp"
#include "Parser.hpp"
#include "Signature.hpp"

using namespace std;

//ctor
ClassicGraph::ClassicGraph(vector<vector<unsigned>> adj){
    this->adjacency=adj;
}


//============== Tree decomposition==================================
//Creates a rooted tree decomposition

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

//===================Brute force===========================================================
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


int ClassicGraph::undominatedDegreeMin(vector<unsigned> S){
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


//==================================  Dijkstra  ============================================================

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

unsigned ClassicGraph::bestDijkstra(unsigned k){

	vector<unsigned> S;
	//Creates first set
	for(unsigned i=0; i<k, i++){
        S.push_back(i);
	}

	unsigned bestDist = makeDijkstra(S);;

	while(kuplet.isInMatrix(matrix, tuple)){
		if(tuple[k].x() == matrix[0].size()){
			printf(".");
		}
		if(kuplet.hasNoWall(matrix, tuple)){
			unsigned tmp = makeDijkstra(tuple);
			if(tmp < bestDist){
				bestDist = tmp;
			}
		}

		tuple = kuplet.nextElement(tuple);
	}
}
//Enumerate Kuplet
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




vector<unsigned> ClassicGraph::listOfVertice(vector<vector<unsigned>> adjacencyList){
    vector<unsigned> listVertice;
    for(unsigned i=0; i<adjacencyList.size(); i++){
            listVertice.push_back(i);
    }
    return listVertice;
}



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
