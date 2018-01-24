#ifndef BAG_H_
#define BAG_H_

#include <vector>

using namespace std;

class Bag{
	// == DATA ==
	vector<unsigned> content;
	
	// == PROTOTYPES ==
	public:
	
	// CONSTRUCTORS
	Bag(); /// Should not be used as it is (only to "prepare" a variable name!)
	Bag(vector<unsigned> content);	// Create a bag with elements only present once
	
	// GETTERS
	vector<unsigned> getContent();
	
	// OTHER
	void add(unsigned e);		// Add element e if not already in content
	void remove(unsigned e);	// Remove element e if present in content
};


#endif

