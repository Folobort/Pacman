#include <vector>

#include "Kuplet.hpp"

using namespace std;

/// -CONSTRUCTOR-
Kuplet::Kuplet(unsigned k, unsigned w, unsigned h){
	this->k = k;
	width = w;
	height = h;
}


/// -TUPLE CONSTRUCTORS-
vector<Point> Kuplet::firstElement (unsigned size){
	vector<Point> first_element;	
		
	if(size == 0){
		return first_element; // empty vector
	}
		
	first_element.push_back(Point(0,0,width,height));
		
	for(unsigned i=0; i<size-1; i++){
		first_element.push_back(first_element[i].next());
	}

	return first_element;
}

vector<Point> Kuplet::nextElement (vector<Point> element){
	vector<Point> next_element;
	
	if(element.size() == 1){
		next_element.push_back(element[0].next());
		return next_element;
	}
		
	//Trouver le plus petit indice i tel que S[i]+1 < S[i+1]
	unsigned i=0;
	unsigned pos1 = element[0].position();
	unsigned pos2 = element[1].position();
	
	while((i<k-2) && (pos2-pos1 == 1)){
		i++;
		pos1 = element[i].position();
		pos2 = element[i+1].position();
	}
	
	if(pos2-pos1 == 1){ // If all elements were contiguous
		i = k-1;		// Then the last one is to be shifted (and not i==k-2)
	}
	
	
	// "retour chariot" des elements avant i s'il y en a
	if(i > 0){
		next_element = firstElement(i);
	}
	
	// shift element i
	next_element.push_back(element[i].next());
	
	// keep the rest as it is
	for(unsigned j=i+1; j<element.size(); j++){
		next_element.push_back(element[j]);
	}	
		
	return next_element;
}


/// -OTHERS-
bool Kuplet::hasNoWall(vector<vector<bool>> matrix, vector<Point> element){
	for(unsigned i=0; i<element.size(); i++){
		if(matrix[element[i].x()][element[i].y()] == false){
			return false;
		}
	}
		
	return true;
}

bool Kuplet::isInMatrix(vector<vector<bool>> matrix, vector<Point> element){	
	for(unsigned i=0; i<element.size(); i++){
		if(element[i].x() < 0 || element[i].x() >= matrix.size() ||
		   element[i].y() < 0 || element[i].y() >= matrix[0].size()){
			return false;
		}
	}
		
	return true;
}





