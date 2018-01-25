#ifndef GRAPH_H_
#define GRAPH_H_

#include <vector>

#include "Point.hpp"
#include "PointFactory.hpp"
#include "Signature.hpp"

using namespace std;

const unsigned INFINITY = -1;

class Graph{
	// == DATA ==
	/// -Graph data-
	vector<vector<bool>> matrix;							// Matrix of the Point state: True for empty, False for wall
	vector<vector<vector<Point>>> neighbors;				// Matrix of neighbors: Each (x,y) represent the list of neighbors
	PointFactory pf = PointFactory(vector<vector<bool>>());	// Point factory, for making the point creation/manipulation less tedious
	
	/// -Dijkstra algorithm, regarding a current set calculation-
	vector<vector<bool>> processed;		// Matrix of state: True if the distance to the set is known for point (x,y)
	vector<vector<unsigned>> dist;		// Matrix of distances: the distance from the point (x,y) to the set
	
	/// -Tree decomposition, regarding a curent signature calculation (by going up the tree)-
	vector<vector<Point>> tree;			// In this particular grid case, the tree decompostion is a path, hence we use a list of bags
	vector<Signature> sigSet;			// The current set of valid signatures up to the addition of the bag (reduced at the smallest possible)
	vector<Point> bestSelectedSet;		// The best dominating set of points dominating the grid

	// == PROTOTYPES ==
	public:
	
	/// -CONSTRUCTORS-
	Graph(vector<vector<bool>> matrix);	// Given a grid, the graph will properly initialize its values

	/// -GETTERS-
	bool isWall(unsigned x, unsigned y);
	bool isWall(Point p);
	unsigned degreeOf(Point point);
	Point lastNoWall();					// Point that is not a wall the most bottom-right positioned in the grid
	
	/// -Dijkstra-
	unsigned makeDijkstra(vector<Point> tuple);	// Returns the longest distance from a point of the grid to the given tuple
	unsigned bestDijkstra(unsigned k);			// Generates all tuples of size k, and returns the smallest distance found using the Dijkstra algorithm on these tuples
	
	/// -Brute force, branch and bound-
	bool isDominatedBy(vector<Point> tuple);			// Returns True if the tuple is a dominating set
	Point getSmallDegreeUndominated(vector<Point> S);	// Given a set S, returns a node with smallest degree that is not is S
	vector<Point> getNeighborsUndominating(Point point, vector<Point> S);	// Returns the neighbors of the point (including itself!) that are not in S
	
	vector<Point> k_dominant(unsigned k, vector<Point> S);	// Determines if there is a set of size k containing S that is a dominating set of the grid. Returns it if found, otherwise an empty vector
	vector<Point> k_dominant(unsigned k);					// Determines if there is a set of size k that is a dominating set of the grid. Returns it if found, otherwise an empty vector
	
	/// -Tree decomposition, signatures-
	vector<Point> firstBag();
	vector<Point> nextBag(vector<Point> bag);
	void treeDecomposition();					// Creates the tree decomposition in this specific case of a grid
	
	Point diff(vector<Point> vp1, vector<Point> vp2);			// Returns the only point that is in the parent bag vp2 and not in the child bag vp1
	void firstSigSet();
	vector<Signature> nextSigSet(Point newPoint);				// Calculates the set of signatures that can be deduced from the current one, considering the new point added
	vector<Signature> nextSigSet();								// Same, but without adding a new point (useful when reducing the last bag)
	vector<Signature> cleanMultipleSig(vector<Signature> sigs); // Removes "worse" signature when duplicatas are found
	
	void solveTree();	// Computes the best dominating set using the tree decomposition and stores it in bestSelectedSet
	
	/// -To string-
	string toString();
};


#endif
