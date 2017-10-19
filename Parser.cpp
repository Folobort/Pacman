#include <fstream>
#include <iostream>
#include <queue>
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <algorithm>

#include "Parser.hpp"

using namespace std;


Parser::Parser(){}	

void Parser::makeMatrix(unsigned width, unsigned height){
	matrix = vector<vector<bool>> (height, vector<bool>(width, false));
}

vector<vector<bool>> Parser::parse(string fileName){
	string line;
		
	ifstream f(fileName);
	if(!f){
		exit(EXIT_FAILURE);
	}
		
	// TYPE
	while(true){
		if(!getline(f, line)){
			exit(EXIT_FAILURE);
		}
			
		if(line[0] == '#' || line.size() == 0 ){ /// Discard comment/empty lines
			continue;
		}
			
		if(line.compare("P1")!=0 && line.compare("P4")!=0){
			exit(EXIT_FAILURE);
		}
			
		//cout << "TYPE" << endl;
		break;
	} 

	// SIZE
	unsigned width = 1;
	unsigned height = 1;
		
	while(true){
		if(!getline(f, line)){
			exit(EXIT_FAILURE);
		}
			
		if(line[0] == '#' || line.size() == 0 ){ /// Discard comment/empty lines
			continue;
		}
			
		stringstream(line) >> width >> height;
			
		makeMatrix(width, height);
			
		break;
	} 		
		
	// DATA
	unsigned x = 0;
	unsigned y = 0;
	while(y < height){			
		if(!getline(f, line)){
			exit(EXIT_FAILURE);
		}
			
		if(line[0] == '#' || line.size() == 0 ){ /// Discard comment/empty lines
			continue;
		}
			
		for(int i=0; i<line.size(); i++){
			matrix[x][y] = (line[i] == '0');
				
			if(++x == width){
				x = 0;
				y++;
			}
		}
	}
		
	return matrix;
}

  
void Parser::showMatrix(vector<vector<bool> > m){
	unsigned height = m.size();
	unsigned width = m[0].size();
		
	for(unsigned i=0; i<height; i++){
		for(unsigned j=0; j<width; j++){
			cout << matrix[i][j];
		}
		cout << endl;
	}
}



