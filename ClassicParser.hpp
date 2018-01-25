#ifndef CLASSICPARSER_H_
#define CLASSICPARSER_H_

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

#include "Node.hpp"

using namespace std;

class ClassicParser{
	// DATA
	
	// PROTOTYPES
	public:
	ClassicParser();	


	vector<Node> parse(string fileName);
  
};


#endif
