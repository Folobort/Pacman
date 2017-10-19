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
	dist = vector<vector<unsigned>> (matrix.size(), vector<unsigned>(matrix[0].size(), INFINITY));
}

bool Graph::isWall(unsigned x, unsigned y){
	return !matrix[x][y];
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



int main(){
	unsigned k = 1;
	
	
	Parser p;
	vector<vector<bool>> matrix = p.parse("lol_map_ascii.pbm");
	//p.showMatrix(matrix);
	
	Graph g = Graph();
	g.setMatrix(matrix);
	
	//cout << Point(0,1).next(512).x() << ", " << Point(0,1).next(512).y() << endl;
	
	unsigned bestDist = g.bestDijkstra(k);
	
	cout << endl;
	cout << "best dist with k = " << k << " : " << bestDist << endl;

	return 0;
}




