#include <fstream>
#include <iostream>
#include <queue>
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <algorithm>
using namespace std;


class graph{
	private:
	vector<vector<bool>> matrix;
	vector<vector<bool>> processed;
	vector<vector<unsigned>> dist;
	
	public:
	void setMatrix(vector<vector<bool>> matrix){
		this.matrix = matrix;
		dist = vector<vector<unsigned>> (matrix.size(), vector<unsigned>(matrix[0].size(), INFINITY));
		
		processed = vector<vector<unsigned>> (matrix.size(), vector<unsigned>(matrix[0].size(), false));
		for(unsigned i=0; i<matrix.size(); i++){
			for(unsigned j=0; j<matrix[0].size(); j++){
				if(matrix[i][j] == false){
					processed[i][j] = true; /// Will ignore walls
				}
			}
		}
	}
	
	void makeDijkstra(vector<Point> tuple){
		vector<Point> toProcessNow;
		
		for(unsigned i=0; i<tuple.size(); i++){
			dist[tuple[i].x()][tuple[i].y()] = 0;
			processed[tuple[i].x()][tuple[i].y()] = true;
			
			toProcessNow.push_back(tuple[i]);
		}
		
		while(toProcessNow.size() > 0){
			Point point = toProcessNow.pop();
			
			
			
		}
		
	
	
	}
	









};


