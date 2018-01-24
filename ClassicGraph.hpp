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

/* 24/01 morning version
class ClassicGraph
{
    // == DATA ==
    //Adjacency List
    vector<vector<unsigned>> adjacency;

    //Distance vector for Dijkstra
    vector<unsigned> dist;

    //Tree decomposition
	vector<vector<unsigned>> tree;

	//For a bag i father[i] is the index of the father in the rooted tree decomposition
	//indexes of the sons for the sons vector
	vector<int> father;
	vector<vector<int>> sons;

	vector<Signature> sigSet;

    public:
        ClassicGraph(vector<vector<unsigned>> adj);

        //Tree decomposition
        bool isComplete(vector<vector<unsigned>> adjacencyList);
        unsigned verticeOfDegreeMin(vector<vector<unsigned>> adjacencyList);
        void treeDecomposition();
        void treeRooting(vector<unsigned> bag);
        vector<vector<unsigned>> updateAdjacency(vector<vector<unsigned>> adjacencyList, vector<unsigned> verticeList, unsigned verticeIndex);
        vector<unsigned> listOfVertice(vector<vector<unsigned>> adjacencyList);
        vector<unsigned> addNewNeighbors(vector<unsigned> neighbors, vector<unsigned> neighborsToAdd);
        //Brute Force
        vector<unsigned> k_dominant(unsigned k, vector<unsigned> S);
        bool isDominating(vector<unsigned> S);
        bool isDominated(vector<unsigned> S, unsigned vertice);
        int undominatedDegreeMin(vector<unsigned> S);
};

*/

class ClassicGraph{
    // == DATA ==
    vector<Node> graph;

    //Distance vector for Dijkstra
    vector<unsigned> dist;

    //Tree decomposition
	TreeDec treeDec;

	vector<Signature> sigSet;

	// == PROTOTYPES ==
    public:
    
    // CONSTRUCTORS
    ClassicGraph(vector<Node> graph);

	// GETTERS
	Node getNodeWithID(unsigned id); // Such node MUST exist in the graph


    //Tree decomposition
    void treeDecomposition();
    
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
    
    
    /*
    bool isComplete(vector<vector<unsigned>> adjacencyList);
    
    vector<unsigned> listOfVertice(vector<vector<unsigned>> adjacencyList);
    */
};

#endif // CLASSICGRAPH_H
