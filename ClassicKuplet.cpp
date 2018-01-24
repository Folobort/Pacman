
#include <vector>

using namespace std;

#include "ClassicKuplet.hpp"

// CONSTRUCTOR
ClassicKuplet::ClassicKuplet(unsigned graphSize){
	this->graphSize = graphSize;
}
	
// OTHER
vector<unsigned> ClassicKuplet::firstElement (unsigned k){
	vector<unsigned> S;
	for(unsigned i=0; i<k; i++){
        S.push_back(i);
	}
	
	return S;
}

vector<unsigned> ClassicKuplet::nextElement (vector<unsigned> S){
	for(unsigned k=1; k<S.size(); k++){
        if((S[k]-S[k-1])!=1){
			// Case 1: the first ID is incremented
			if(k==1){
				S[k-1]++;
				return S;
			}
			// Case 2: another ID (not the last) is incremented, all smaller IDs are reseted
			else{
				vector<unsigned> S2 = firstElement(k-2);
				S2.push_back(S[k-1]+1);
				for(unsigned l=k; l<S.size();l++){
					S2.push_back(S[l]);
				}
				return S2;
			}
        }
    }
    // Case 3: the last ID is incremented, all other IDs are reseted
    vector<unsigned> S2 = firstElement(S.size()-1);
    S2.push_back(S[S.size()-1]+1);
    
    return S;
}


