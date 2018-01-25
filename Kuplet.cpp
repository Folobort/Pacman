#include <fstream>
#include <iostream>
#include <queue>
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <algorithm>

#include "Kuplet.hpp"

using namespace std;

// CONSTRUCTOR
Kuplet::Kuplet(unsigned k, unsigned w, unsigned h){
	this->k = k;
	width = w;
	height = h;
}


// BAG CONSTRUCTORS
vector<Point> Kuplet::firstElement (unsigned size){
	vector<Point> first_element;	
		
	if(size == 0){
		return first_element; // empty vector
	}
		
	first_element.push_back(Point(0,0,width,height));
		
	for(unsigned i=0; i<size; i++){
		first_element.push_back(first_element[i].next());
	}

	return first_element;
}

/*
vector<Point> Kuplet::nextElement (vector<Point> element){
	vector<Point> next_element;
	
	if(element.size() == 1){
		next_element.push_back(element[0].next());
		return next_element;
	}
		
	//Trouver le plus petit indice i tel que S[i]+1 < S[i+1]
	int i = 0;
	int s = 0; int t = 0;
	do {
		s = element[i].position();
		t = element[++i].position();
	} while((t-s == 1) && (i < k-1)); /// tant que consécutif et non dernier
		
	if(i == k-1){ /// cas tous consécutifs
		i = k;
		if(element[k-1].isLast()){
			exit(EXIT_FAILURE); // CHANGE LATER (cas fin)
		}
	}
		
	next_element = firstElement(i-1); /// "retour chariot"
	next_element.push_back(element[i-1].next()); /// décalage
	for (int p = i; p<k; p++){ /// copie du reste
		next_element.push_back(element[p]);
	}
		
	return next_element;
}
*/

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
	
	while((i<k-1) && (pos2-pos1 == 1)){
		i++;
		pos1 = element[i].position();
		pos2 = element[i+1].position();
	}
	
	// "retour chariot" des elements avant i s'il y en a
	if(i > 0){
		next_element = firstElement(i-1);
	}
	
	// shift element i
	next_element.push_back(element[i].next());
	
	// keep the rest as it is
	for(unsigned j=i+1; j<element.size(); j++){
		next_element.push_back(element[j]);
	}
	
	return next_element;
}


// OTHERS

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





