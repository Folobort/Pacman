#ifndef PARSER_H_
#define PARSER_H_

#include <string>
#include <vector>

using namespace std;

class Parser{
	// DATA
	vector<vector<bool>> matrix;	// Space to work on
	
	// PROTOTYPES
	public:
	
	/// -CONSTRUCTOR-
	Parser();

	/// -Parser-
	void makeMatrix(unsigned width, unsigned height);	// Creates a matrix of specified dimensions

	vector<vector<bool> > parse(string fileName);		// Parses the specified pbm file into a matrix
  
	/// -To string-
	void showMatrix(vector<vector<bool>> m);			// Displays matrix content
};


#endif
