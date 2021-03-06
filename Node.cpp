#include <fstream>
#include <iostream>
#include <queue>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>

using namespace std;

#include "Node.hpp"

// CONSTRUCTORS
Node::Node(){
	this->id = 0;
}

Node::Node(unsigned id){
	this->id = id;
}

Node::Node(unsigned id, vector<unsigned> neighborsID){
	this->id = id;
	this-> neighborsID = neighborsID;
}
	
// GETTERS
unsigned Node::getID(){
	return id;
}
vector<unsigned> Node::getNeighborsID(){
	return neighborsID;
}
unsigned Node::getDeg(){
	return neighborsID.size();
}
	
// OTHER
void Node::addNeighbor(unsigned v){
	// Stop if node itself
	if(id == v){return;}
	
	// Normal case
	vector<unsigned>::iterator it = find(neighborsID.begin(), neighborsID.end(), v);
	
	if(it == neighborsID.end()){
		neighborsID.push_back(v);
	}
}

void Node::addNeighbors(vector<unsigned> v){
	for(unsigned i=0; i<v.size(); i++){
		addNeighbor(v[i]);
	}
}

void Node::removeNeighbor(unsigned v){
	vector<unsigned>::iterator it = find(neighborsID.begin(), neighborsID.end(), v);
	
	if(it != neighborsID.end()){
		neighborsID.erase(it);
	}
}



// TO STRING

void Node::toString(){
	cout << " Node : " << id << " neighbors : ";
	for(unsigned i=0; i<neighborsID.size(); i++){
		cout << neighborsID[i] << " - ";
	}
	cout << endl;
} 
