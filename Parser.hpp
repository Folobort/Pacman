#ifndef PARSER_H_
#define PARSER_H_

#include <fstream>
#include <iostream>
#include <queue>
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <algorithm>

using namespace std;

class Parser{
	// DATA
	vector<vector<bool>> matrix;
	
	// PROTOTYPES
	public:
	Parser();	

	void makeMatrix(unsigned width, unsigned height);

	vector<vector<bool> > parse(string fileName);
  
	void showMatrix(vector<vector<bool>> m);
};


#endif
