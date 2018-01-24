
#include <vector>
#include <algorithm>

using namespace std;

#include "Bag.hpp"

// CONSTRUCTORS
Bag::Bag(){}

Bag::Bag(vector<unsigned> content){
	this->content = content;
}
	
// GETTERS
vector<unsigned> Bag::getContent(){
	return content;
}
	
// OTHER
void Bag::add(unsigned e){
	vector<unsigned>::iterator it = find(content.begin(), content.end(), e);
	
	if(it == content.end()){
		content.push_back(e);
	}
}

void Bag::remove(unsigned e){
	vector<unsigned>::iterator it = find(content.begin(), content.end(), e);
	
	if(it != content.end()){
		content.erase(it);
	}
}





