#ifndef CLASSICKUPLET_H_
#define CLASSICKUPLET_H_

#include <vector>

using namespace std;

class ClassicKuplet{
	// == DATA ==
	unsigned graphSize;
	
	// == PROTOTYPES ==
	public:
	
	// CONSTRUCTOR
	ClassicKuplet(unsigned graphSize);
	
	// OTHER
	vector<unsigned> firstElement (unsigned k);
	vector<unsigned> nextElement (vector<unsigned> element);
};


#endif
