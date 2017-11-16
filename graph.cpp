#include <fstream>
#include <iostream>
#include <queue>
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <algorithm>

#include "Graph.hpp"
#include "Parser.hpp"
#include "Kuplet.hpp"

using namespace std;


Graph::Graph(){}

void Graph::setMatrix(vector<vector<bool>> matrix){
	this->matrix = matrix;

	neighbors = vector<vector<vector<Point>>> (matrix.size(), vector<vector<Point>> (matrix[0].size()));

	for(unsigned x=0; x<matrix.size(); x++){
		for(unsigned y=0; y<matrix[0].size(); y++){
			if(x > 0 && !isWall(x,y)){ // If not on left edge nor wall, then notify left neighbor
				neighbors[x-1][y].push_back(Point(x,y));
			}
			if(x < matrix.size()-1 && !isWall(x,y)){
				neighbors[x+1][y].push_back(Point(x,y));
			}
			if(y < 0 && !isWall(x,y)){
				neighbors[x][y-1].push_back(Point(x,y));
			}
			if(y < matrix[0].size()-1 && !isWall(x,y)){
				neighbors[x][y+1].push_back(Point(x,y));
			}
		}
	}

	dist = vector<vector<unsigned>> (matrix.size(), vector<unsigned>(matrix[0].size(), INFINITY));
}

void Graph::treeDecomposition(){
    vector<Point> bag = firstBag();
    
    //Add bag to the tree decomposition
    tree.push_back(bag);
    cout << "First bag" << endl;
	
    //Update bag
    Point lastValid = lastNoWall();
    cout << "Last valid: (" << lastValid.x() << "," << lastValid.y() << ")" << endl;
    /// (42,512)
    
    unsigned k = 0;
    while(!(bag.back().equals(lastValid))){
		bag = nextBag(bag);
		tree.push_back(bag);
		
		k++;
		cout << "Next bag " << k << ", bag.back = (" << bag.back().x() << "," << bag.back().y() << ")" << endl;
	}
}

vector<Point> Graph::firstBag(){
	vector<Point> bag; //a bag contain each step of the tree decomposition starting with first line
    Point p = Point(0,0);
    while(isWall(p)){
		p=p.next(matrix.size());
	}
    bag.push_back(p); // first no-wall Point
    while(bag.size()<matrix.size()+1){
        p = p.next(matrix.size());
        while(isWall(p)){
			p=p.next(matrix.size());
		}
        bag.push_back(p);
    }
	
	return bag;
}

vector<Point> Graph::nextBag(vector<Point> bag){
	Point nextAdd = bag.back().next(matrix.size());
	while(!nextAdd.isLast(matrix.size(), matrix[0].size()) && !isWall(nextAdd)){
		nextAdd = nextAdd.next(matrix.size());
	}
	
	/* Should not be called normally
	if(nextAdd.isLast && nextAdd.isWall){
		return bag;
		// bag was last valid bag, abort somehow
	}
	*/
	
	cout << "NextAdd: (" << nextAdd.x() << "," << nextAdd.y() << ")" << endl;
	
	
	
	bag.erase(bag.begin());
	bag.push_back(nextAdd);
	
	return bag;
}



Point Graph::lastNoWall(){
	Point lastValid = Point(matrix.size()-1, matrix[0].size()-1);
	
	while(isWall(lastValid)){
		lastValid = lastValid.before(matrix.size());
	}

	return lastValid;
}



bool Graph::isWall(unsigned x, unsigned y){
	return !matrix[x][y];
}

bool Graph::isWall(Point p){
	return isWall(p.x(), p.y());
}



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
			toProcessNow.push_back(Point(x-1, y));
		}
		if(x < matrix.size()-1 && !processed[x+1][y]){ /// Process RIGHT
			processed[x+1][y] = true;
			dist[x+1][y] = dist[x][y] + 1;
			toProcessNow.push_back(Point(x+1, y));
		}
		if(y > 0 && !processed[x][y-1]){ /// Process UP
			processed[x][y-1] = true;
			dist[x][y-1] = dist[x][y] + 1;
			toProcessNow.push_back(Point(x, y-1));
		}
		if(y < matrix[0].size()-1 && !processed[x][y+1]){ /// Process DOWN
			processed[x][y+1] = true;
			dist[x][y+1] = dist[x][y] + 1;
			toProcessNow.push_back(Point(x, y+1));
		}
	}

	return maxDist;
}

unsigned Graph::bestDijkstra(unsigned k){

	Kuplet kuplet = Kuplet(k);

	vector<Point> tuple = kuplet.firstElement(k, matrix.size(), matrix[0].size());
	while(!kuplet.hasNoWall(matrix, tuple)){
		tuple = kuplet.nextElement(tuple, matrix.size(), matrix[0].size());
	}
	unsigned bestDist = makeDijkstra(tuple);;

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

		tuple = kuplet.nextElement(tuple,matrix.size(), matrix[0].size());
	}
}

unsigned Graph::degreeOf(Point point){
	return neighbors[point.x()][point.y()].size();
}


Point Graph::getSmallDegreeUndominated(vector<Point> S){
	unsigned smallestDeg = 5;
	Point point = Point(0,0);

	for(unsigned x=0; x<matrix.size(); x++){
		for(unsigned y=0; y<matrix[0].size(); y++){
			if(Point(x,y).isInVector(S) || isWall(x,y)){
				continue;
			}

			unsigned deg = degreeOf(Point(x,y));
			if(deg < smallestDeg){
				smallestDeg = deg;
				point = Point(x,y);
			}
		}
	}

	return point;
}

vector<Point> Graph::getNeighbors(Point point){
	vector<Point> neighborExtended = neighbors[point.x()][point.y()];
	neighborExtended.push_back(point);

	return neighborExtended;
}




vector<Point> Graph::getNeighborsUndominating(Point point, vector<Point> S){
	vector<Point> neighbors = getNeighbors(point);
	vector<Point> neighborsUndominating;

	for(unsigned i=0; i<neighbors.size(); i++){
		if(!neighbors[i].isInVector(S)){
			neighborsUndominating.push_back(neighbors[i]);
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

	cout << "======================================" << endl;
	cout << "v (" << degreeOf(v) << "): " << v.x() << " " << v.y() << endl;

	vector<Point> neighbors = getNeighborsUndominating(v, S);

	for(unsigned i=0; i<neighbors.size(); i++){

		//cout << "N: " << neighbors[i].x() << ", " << neighbors[i].y() << endl;

		vector<Point> S2 = S;
		S2.push_back(neighbors[i]);

		if(k_dominant(k, S2).size() != 0){
			return S2;
		}
	}

	vector<Point> emptyV;
	return emptyV; /// All sub-S failed
}





int main(){
	unsigned k = 10;


	Parser p;
	vector<vector<bool>> matrix = p.parse("lol_map_ascii.pbm");
	//p.showMatrix(matrix);

	Graph g = Graph();
	g.setMatrix(matrix);

	//unsigned bestDist = g.bestDijkstra(k);

	//cout << endl;
	//cout << "best dist with k = " << k << " : " << bestDist << endl;

/* Branch and bound
	vector<Point> emptyV;
	vector<Point> kDom = g.k_dominant(k, emptyV);

	if(kDom.size() != 0){
		cout << "Works with set of size " << k << ":" << endl;
		for(unsigned i=0; i<k-1; i++){
			cout << kDom[i].toString() << ", ";
		}
		cout << kDom[k-1].toString() << endl;
	} else{
		cout << "Does not work with set of size " << k << endl;
	}
*/

	g.treeDecomposition();



	return 0;
}




