#include <fstream>
#include <iostream>
#include <queue>
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <algorithm>
#include <iterator>
#include <sstream>

#include "ClassicParser.hpp"
#include "Node.hpp"

using namespace std;


ClassicParser::ClassicParser(){}	


vector<Node> ClassicParser::parse(string fileName){
	string line;
	string delimiter=": ";
	string delimiter2=" ";
	unsigned id;
	vector<unsigned> neighbors;
	vector<Node> graph;
	
	ifstream f(fileName);
	if(!f){
		exit(EXIT_FAILURE);
	}
		
	while(getline(f, line)){
		
		//Extract line
					
		if(line[0] == '/' || line.size() == 0 ){ /// Discard comment/empty lines
			continue;
		}

		//Extract id
		id=stoi(line.substr(0, line.find(delimiter)), nullptr);
		Node u= Node(id);
		//Delete id in line
		line.erase(0, line.find(delimiter) + delimiter.length());
		
		//Extract Neighbors
		
		size_t pos = 0;
		std::string neighbor;
		while ((pos = line.find(delimiter2)) != string::npos) {
			neighbor = line.substr(0, pos);
			u.addNeighbor(stoi(neighbor, nullptr));
			line.erase(0, pos + delimiter2.length());
		}
		
		u.addNeighbor(stoi(line, nullptr));
/*	
		istringstream iss(line);
		vector<string> otherIds;
		copy(istream_iterator<string>(iss), istream_iterator<string>(), back_inserter(otherIds));
		for(unsigned i=0; i<otherIds.size(); i++){
			u.addNeighbor(stoi(otherIds[i], nullptr));
		}
*/		
		graph.push_back(u);
		
		
	} 
	return graph;
}



