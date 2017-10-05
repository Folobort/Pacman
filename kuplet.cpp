/*
 * kuplet.cpp
 *
 *  Created on: 24 sept. 2017
 *      Author: louis
 */

#include <fstream>
#include <iostream>
#include <queue>
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <algorithm>

#include "Point.cpp"

///#include "kuplet.h"

using namespace std;

class kuplet{
	private:
	unsigned k;

	public:
	kuplet(unsigned k){
		this->k = k;
	}

	vector<Point> nextElement (vector<Point> element, unsigned width, unsigned height){
		vector<Point> next_element;
		
		//Trouver le plus petit indice i tel que S[i]+1<S[i+1]
		int i = 0;
		int s = 0; int t = 0;
		do {
			s = element[i].position(width);
			t = element[++i].position(width);
		} while((t-s == 1) && (i < k-1)); /// tant que cons�cutif et non dernier
		
		if(i == k-1){ /// cas tous cons�cutifs
			i = k;
			if(element[k-1].last(width, height)){
				exit(EXIT_FAILURE); // CHANGE LATER (cas fin)
			}
		}
		
		next_element = firstElement(i-1, width, height); /// "retour chariot"
		next_element.push_back(element[i-1].next(width)); /// d�calage
		for (int p = i; p<k; p++){ /// copie du reste
			next_element.push_back(element[p]);
		}
		
		return next_element;
	}

	vector<Point> firstElement (unsigned size, unsigned width, unsigned height){
		vector<Point> first_element;	
		
		if(size == 0){
			return first_element; // empty vector
		}
		
		first_element.push_back(Point(0,0));
		
		for(unsigned i=0; i<size; i++){
			first_element.push_back(first_element[i].next(width));
		}

		return first_element;
	}
	
	bool checkWall (vector<vector<bool>> matrix, vector<Point> element){
		for(unsigned i=0; i<element.size(); i++){
			if(matrix[element[i].x()][element[i].y()] == false){
				return false;
			}
		}
		
		return true;
	}

};

int main(){
	return 0;
}


