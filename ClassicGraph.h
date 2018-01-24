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

#include "Signature.hpp"

using namespace std;

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

#endif // CLASSICGRAPH_H
