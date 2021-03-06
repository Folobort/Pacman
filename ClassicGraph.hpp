#ifndef CLASSICGRAPH_H
#define CLASSICGRAPH_H

#include <fstream>
#include <iostream>
#include <queue>
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <algorithm>

#include "Node.hpp"
#include "TreeDec.hpp"
#include "Signature.hpp"
#include "GraphAux.hpp"

using namespace std;

class ClassicGraph{
    // == DATA ==
    vector<Node> graph;

    //Distance vector for Dijkstra
    vector<unsigned> dist;

    //Tree decomposition
	TreeDec treeDec;

	vector<unsigned> bestSelectedSet;	// The best dominating set of nodes dominating the grid

	// == PROTOTYPES ==
    public:
    
    // CONSTRUCTORS
    ClassicGraph(vector<Node> graph);

	// GETTERS
	Node getNodeWithID(unsigned id); // Such node MUST exist in the graph


    //Tree decomposition
    void treeDecomposition();
    void solveTree();
    
    //other stuff to sort
    
    //Brute Force
    vector<unsigned> k_dominant(unsigned k, vector<unsigned> S);
    unsigned undominatedDegreeMin(vector<unsigned> S);
    bool isDominating(vector<unsigned> S);
    bool isDominated(vector<unsigned> S, unsigned id);
    bool isDominated(vector<unsigned> S, Node u);
    
    
    // Dijkstra
    unsigned Dijkstra(vector<unsigned> S);
    vector<unsigned> bestDijkstra(unsigned k);
    
    //TO STRING
    string toString();
    
    
    
    /*
    bool isComplete(vector<vector<unsigned>> adjacencyList);
    
    vector<unsigned> listOfVertice(vector<vector<unsigned>> adjacencyList);
    */
};

#endif // CLASSICGRAPH_H
