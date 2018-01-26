#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>

#include "Graph.hpp"
#include "Parser.hpp"
#include "Kuplet.hpp"
#include "Point.hpp"
#include "PointFactory.hpp"
#include "Signature.hpp"
#include "ClassicParser.hpp"
#include "ClassicGraph.hpp"
#include "Node.hpp"

using namespace std;

/// -CONSTRUCTORS-
Graph::Graph(vector<vector<bool>> matrix){
	this->matrix = matrix;
	
	// Set PointFactory
	pf = PointFactory(matrix);

	neighbors = vector<vector<vector<Point>>> (matrix.size(), vector<vector<Point>> (matrix[0].size()));

	for(unsigned x=0; x<matrix.size(); x++){
		for(unsigned y=0; y<matrix[0].size(); y++){
			if(!isWall(x,y)){
				if(x > 0){ // If not on left edge nor wall, then notify left neighbor
					neighbors[x-1][y].push_back(pf.mkPoint(x,y));
				}
				if(x < matrix.size()-1){
					neighbors[x+1][y].push_back(pf.mkPoint(x,y));
				}
				if(y > 0){
					neighbors[x][y-1].push_back(pf.mkPoint(x,y));
				}
				if(y < matrix[0].size()-1){
					neighbors[x][y+1].push_back(pf.mkPoint(x,y));
				}
			}
		}
	}

	dist = vector<vector<unsigned>> (matrix.size(), vector<unsigned>(matrix[0].size(), INFINITY));
}


/// -GETTERS-
bool Graph::isWall(unsigned x, unsigned y){
	return !matrix[x][y];
}
bool Graph::isWall(Point p){
	return isWall(p.x(), p.y());
}

unsigned Graph::degreeOf(Point point){
	return neighbors[point.x()][point.y()].size();
}

Point Graph::lastNoWall(){
	Point lastValid = pf.mkPointLast();
	
	while(isWall(lastValid)){
		lastValid = lastValid.before();
	}
	return lastValid;
}


/// -Dijkstra-
unsigned Graph::makeDijkstra(vector<Point> tuple){	
	processed = vector<vector<bool> > (matrix.size(), vector<bool>(matrix[0].size(), false));

	vector<Point> toProcessNow;
	for(unsigned i=0; i<tuple.size(); i++){ /// Set tuple elements to dist 0
		dist[tuple[i].x()][tuple[i].y()] = 0;
		processed[tuple[i].x()][tuple[i].y()] = true;

		toProcessNow.push_back(tuple[i]);
	}
	
	unsigned maxDist = 0;
	while(toProcessNow.size() > 0){
		Point point = toProcessNow.front();
		toProcessNow.erase(toProcessNow.begin()); // pop_front()
		unsigned x = point.x();
		unsigned y = point.y();

		if(dist[x][y] > maxDist){
			maxDist = dist[x][y];
		}

		if(isWall(x,y)){ /// Process WALL
			processed[x][y] = true;
		}

		if(x > 0 && !processed[x-1][y]){ /// Process LEFT
			processed[x-1][y] = true;
			dist[x-1][y] = dist[x][y] + 1;
			toProcessNow.push_back(pf.mkPoint(x-1, y));
		}
		if(x < matrix.size()-1 && !processed[x+1][y]){ /// Process RIGHT
			processed[x+1][y] = true;
			dist[x+1][y] = dist[x][y] + 1;
			toProcessNow.push_back(pf.mkPoint(x+1, y));
		}
		if(y > 0 && !processed[x][y-1]){ /// Process UP
			processed[x][y-1] = true;
			dist[x][y-1] = dist[x][y] + 1;
			toProcessNow.push_back(pf.mkPoint(x, y-1));
		}
		if(y < matrix[0].size()-1 && !processed[x][y+1]){ /// Process DOWN
			processed[x][y+1] = true;
			dist[x][y+1] = dist[x][y] + 1;
			toProcessNow.push_back(pf.mkPoint(x, y+1));
		}
	}

	return maxDist;
}

unsigned Graph::bestDijkstra(unsigned k){
	Kuplet kuplet = Kuplet(k, matrix.size(), matrix[0].size());

	vector<Point> tuple = kuplet.firstElement(k);
	
	while(!kuplet.hasNoWall(matrix, tuple)){
		tuple = kuplet.nextElement(tuple);
	}
	
	unsigned bestDist = makeDijkstra(tuple);
	cout << "first bestDist : " << bestDist << endl;

	while(kuplet.isInMatrix(matrix, tuple)){
		if(kuplet.hasNoWall(matrix, tuple)){			
			unsigned tmp = makeDijkstra(tuple);
			if(tmp < bestDist){
				bestDist = tmp;
				cout << "new bestDist : " << bestDist << endl;
			}
		}

		tuple = kuplet.nextElement(tuple);
	}
	
	return bestDist;
}


/// -Brute force, branch and bound-
bool Graph::isDominatedBy(vector<Point> tuple){
		vector<vector<bool>> inTuple = vector<vector<bool>> (matrix.size(), vector<bool>(matrix[0].size(), false));

		for(unsigned i=0; i<tuple.size(); i++){
			inTuple[tuple[i].x()][tuple[i].y()] = true;
		}

		for(unsigned x=0; x<matrix.size(); x++){
			for(unsigned y=0; y<matrix[0].size(); y++){
				/// Wall, in the set, or next to an element of the set
				if(isWall(x,y)) continue;
				if(inTuple[x][y]) continue;
				if(x > 0 && inTuple[x-1][y]) continue;
				if(x < matrix.size()-1 && inTuple[x+1][y]) continue;
				if(y > 0 && inTuple[x][y-1]) continue;
				if(y < matrix[0].size()-1 && inTuple[x][y+1]) continue;

				return false;
			}
		}

		return true;
	}

Point Graph::getSmallDegreeUndominated(vector<Point> S){
	unsigned smallestDeg = 5; // In a grid, only 4 neighbors
	Point point = pf.mkPoint(0,0);

	for(unsigned x=0; x<matrix.size(); x++){
		for(unsigned y=0; y<matrix[0].size(); y++){
			if(pf.mkPoint(x,y).isInVector(S) || isWall(x,y)){
				continue;
			}

			unsigned deg = degreeOf(pf.mkPoint(x,y));
			if(deg < smallestDeg){
				smallestDeg = deg;
				point = pf.mkPoint(x,y);
			}
		}
	}

	return point;
}

vector<Point> Graph::getNeighborsUndominating(Point point, vector<Point> S){
	vector<Point> neighborsExtended = neighbors[point.x()][point.y()];
	neighborsExtended.push_back(point);
	
	vector<Point> neighborsUndominating;

	for(unsigned i=0; i<neighborsExtended.size(); i++){
		if(!neighborsExtended[i].isInVector(S)){
			neighborsUndominating.push_back(neighborsExtended[i]);
		}
	}

	return neighborsUndominating;
}

vector<Point> Graph::k_dominant(unsigned k, vector<Point> S){
	if(S.size() == k){
		if(isDominatedBy(S)){
			return S;
		}
		vector<Point> emptyV;
		return emptyV; /// Empty vector on failure
	}

	Point v = getSmallDegreeUndominated(S);

	vector<Point> neighbors = getNeighborsUndominating(v, S);

	for(unsigned i=0; i<neighbors.size(); i++){
		vector<Point> S2 = S;
		S2.push_back(neighbors[i]);

		vector<Point> S3 = k_dominant(k, S2);
		if(S3.size() != 0){
			return S3;
		}
	}

	vector<Point> emptyV;
	return emptyV; /// All sub-S failed
}
vector<Point> Graph::k_dominant(unsigned k){
	vector<Point> emptySet;
	
	return k_dominant(k, emptySet);
}


/// -Tree decomposition, signatures-
vector<Point> Graph::firstBag(){
	vector<Point> bag; //a bag contain each step of the tree decomposition starting with first line
    Point p = pf.mkPoint(0,0);
    while(isWall(p)){
		p=p.next();
	}
    bag.push_back(p); // first no-wall Point
    
    while(bag.size() < matrix.size()+1){
        do{
			p=p.next();
		} while(isWall(p));
		
        bag.push_back(p);
    }
	
	return bag;
}

vector<Point> Graph::nextBag(vector<Point> bag){
	Point nextAdd = bag.back().next();
	
	while(!nextAdd.isLast() && isWall(nextAdd)){
		nextAdd = nextAdd.next();
	}
	
	bag.erase(bag.begin());
	bag.push_back(nextAdd);
	
	return bag;
}

void Graph::treeDecomposition(){
    vector<Point> bag = firstBag();
    
    //Add bag to the tree decomposition
    tree.push_back(bag);
	
    //Update bag
    Point lastValid = lastNoWall();
    
    unsigned k = 0;
    while(!(bag.back().equals(lastValid))){
		bag = nextBag(bag);
		tree.push_back(bag);
	}
	
	//cout << "Created " << tree.size() << " bags!" << endl;
}


Point Graph::diff(vector<Point> vp1, vector<Point> vp2){
	for(unsigned i=0; i<vp2.size(); i++){
		if(!vp2[i].isInVector(vp1)){
			return vp2[i];
		}
	}
	
	cout << "ERR DIFF PLZ STOP" << endl; /// balancer une exception dans le cas d'égalité
	return pf.mkPoint(0,0);
}

void Graph::firstSigSet(){
	vector<Point> firstBag = tree[tree.size()-1];
	
	Signature newSig = Signature(firstBag);
	sigSet.push_back(newSig);
}

vector<Signature> Graph::nextSigSet(Point newPoint){
	vector<Signature> nextSigs;
	
	for(unsigned i=0; i<sigSet.size(); i++){
		vector<Signature> sigSetElem = sigSet[i].update(newPoint, neighbors);
		for(unsigned k=0; k<sigSetElem.size(); k++){
			nextSigs.push_back(sigSetElem[k]);
		}
	}
	
	return cleanMultipleSig(nextSigs);
}
vector<Signature> Graph::nextSigSet(){
	vector<Signature> nextSigs;
	
	for(unsigned i=0; i<sigSet.size(); i++){
		vector<Signature> sigSetElem = sigSet[i].update(neighbors);
		for(unsigned k=0; k<sigSetElem.size(); k++){
			nextSigs.push_back(sigSetElem[k]);
		}
	}
	
	return cleanMultipleSig(nextSigs);
}

vector<Signature> Graph::cleanMultipleSig(vector<Signature> sigs){
	vector<Signature> cleanedSigs;
	vector<bool> keepIndex(sigs.size(), true);
	
	for(unsigned i=0; i<sigs.size(); i++){
		if(keepIndex[i]){
			for(unsigned j=i+1; j<sigs.size(); j++){
				if(keepIndex[j] && sigs[i].equals(sigs[j])){ // same apparent signature
					if(sigs[i].getSelected().size() <= sigs[j].getSelected().size()){
						keepIndex[j] = false;
					}else{
						keepIndex[i] = false;
					}
				}
			}
		}
	}
	
	for(unsigned i=0; i<sigs.size(); i++){
		if(keepIndex[i]){
			cleanedSigs.push_back(sigs[i]);
		}
	}
	
	return cleanedSigs;
}

void Graph::solveTree(){
	firstSigSet();

	// Compute signatures up the tree-path
	for(unsigned i=tree.size()-1; i>0; i--){ 
		Point newPoint = diff(tree[i],tree[i-1]);
	
		sigSet = nextSigSet(newPoint);
	}
	
	///cout << "SIGNATURES COUNT : " << sigSet.size() << endl;
	
	// Compute the signatures on the last bag (removing one point at a time)
	for(unsigned i=0; i<tree[0].size(); i++){
		sigSet = nextSigSet();

		///cout << "=================" << endl;
		///cout << "SIGNATURES COUNT " << i << " : " << sigSet.size() << endl;
	}
	
	// Select best signature among the ones left
	Signature finalSig = sigSet[0];
	for(unsigned i=1; i<sigSet.size(); i++){
		if(sigSet[i].isBetterFinalSig(finalSig)){
			finalSig = sigSet[i];
		}
	}
	
	// Get the selected set from that signature
	vector<Point> finalSet = finalSig.getSelected();
	for(unsigned i=0; i<finalSet.size(); i++){
		bestSelectedSet.push_back(finalSet[i]);
	}
}


/// -To string-
string Graph::toString(){
	stringstream ss;
	unsigned w = matrix.size();
	unsigned h = matrix[0].size();
	
	ss << "graph G {" << endl;
	
	// Force all positions (for faithful display)
	for(unsigned y=0; y<h; y++){
		for(unsigned x=0; x<w; x++){
			if(matrix[x][y]){ // Not a wall
				unsigned pos1 = y*w + x;
				string shapeStr;
				if(pf.mkPoint(x,y).isInVector(bestSelectedSet)){
					shapeStr = "shape = box";
				} else{
					shapeStr = "shape = circle";
				}
				
				ss << pos1 << "[pos = \"" << x << "," << (h-1)-y << "!\" " << shapeStr << "]" << ";" << endl;
			}
		}
	}
	
	// Add edges
	for(unsigned y=0; y<h; y++){
		for(unsigned x=0; x<w; x++){
			if(matrix[x][y]){ // Not a wall
				unsigned pos1 = y*w + x;
				for(unsigned i=0; i<neighbors[x][y].size(); i++){
					unsigned pos2 = neighbors[x][y][i].y() * w + neighbors[x][y][i].x();
					if(pos1 < pos2){
						ss << pos1 << "--" << pos2 << ";" << endl;
					}
				}
			}
		}
	}
	
	ss << "}";
	
	return ss.str();
}


/// -MAIN CALL-
int main(){
/// GRID MAIN	
/*
	unsigned k = 1;

	Parser p;
	vector<vector<bool>> matrix = p.parse("maps/lol_map_ascii_femtoTest.pbm");

	Graph g = Graph(matrix);

	/// Dijkstra

	unsigned bestDist = g.bestDijkstra(k);

	cout << "===============" << endl;
	cout << "best dist with k = " << k << " : " << bestDist << endl;
	cout << "===============" << endl;

	/// Branch and bound

	vector<Point> kDom = g.k_dominant(k);

	if(kDom.size() != 0){
		cout << "There is a dominating set of size " << k << ":" << endl;
		for(unsigned i=0; i<k-1; i++){
			cout << kDom[i].toString() << ", ";
		}
		cout << kDom[k-1].toString() << endl;
	} else{
		cout << "Does not dominate with any set of size " << k << endl;
	}

	/// Tree Decomposition
	g.treeDecomposition();
	g.solveTree();

	cout << g.toString() << endl;
	
	return 0;
*/

/// CLASSIC MAIN
	ClassicParser p;
	vector<Node> graph=p.parse("testparser.txt");
	ClassicGraph g= ClassicGraph(graph);
	
	/*
	cout << "Graph from gengraph" << endl;
	cout << g.toString() << endl;
	*/
	
	g.treeDecomposition();
	g.toString();
	g.solveTree();
/*	
	vector<unsigned> S;
	vector<unsigned> dominant = g.k_dominant(3, S);
	
	cout << "Sommet dominants : " << endl;
	for(unsigned j=0; j<dominant.size(); j++){
		cout << dominant[j] << endl;
	}
*/
	

	return 0;
}




