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
	vector<vector<unsigned>> dist;
	
	public:
	void setMatrix(vector<vector<bool>> matrix){
		this.matrix = matrix;
		dist = vector<vector<unsigned>> (matrix.size(), vector<unsigned>(matrix[0].size(), infinity));
	}
	
	void makeDijkstra(vector<Point> tuple){
		vector<Point> toDo;
	
		for(int i=0; i<tuple.size(); i++){
			dist[tuple[i].x()][tuple[i].y()] = 0;
		}
	
		
	
	
	}
	









};


