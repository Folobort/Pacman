#include <fstream>
#include <iostream>
#include <queue>
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <algorithm>
using namespace std;


class parser{
	private:
	vector<vector<bool>> matrix;
	
	
	public:
	parser(){}	

	void makeMatrix(unsigned width, unsigned height){
		matrix = vector<vector<bool>> (height, vector<bool>(width, false));
	}

	vector<vector<bool> > parse(string fileName){
		string line;
		
		ifstream f(fileName);
		if(!f){
			exit(EXIT_FAILURE);
		}
		
		// TYPE
		while(true){
			if(!getline(f, line)){
				exit(EXIT_FAILURE);
			}
			
			if(line[0] == '#' || line.size() == 0 ){ /// Discard comment/empty lines
				continue;
			}
			
			if(line.compare("P1")!=0 && line.compare("P4")!=0){
				exit(EXIT_FAILURE);
			}
			
			cout << "TYPE" << endl;
			break;
		} 

		// SIZE
		unsigned width = 1;
		unsigned height = 1;
		
		while(true){
			if(!getline(f, line)){
				exit(EXIT_FAILURE);
			}
			
			if(line[0] == '#' || line.size() == 0 ){ /// Discard comment/empty lines
				continue;
			}
			
			stringstream(line) >> width >> height;
			
			cout << width << ", " << height << endl;
			
			makeMatrix(width, height);
			
			cout << "SIZE" << endl;
			
			break;
		} 		
		
		// DATA
		unsigned x = 0;
		unsigned y = 0;
		while(y < height){			
			if(!getline(f, line)){
				exit(EXIT_FAILURE);
			}
			
			if(line[0] == '#' || line.size() == 0 ){ /// Discard comment/empty lines
				continue;
			}
			
			for(int i=0; i<line.size(); i++){
				matrix[x][y] = (line[i] == '0');
				
				if(++x == width){
					x = 0;
					y++;
					
					cout << endl;
				}
			}
		}
		
		cout << "DATA" << endl; 
		
		return matrix;
	}

  
	void showMatrix(vector<vector<bool> > m){
		unsigned height = m.size();
		unsigned width = m[0].size();
		
		for(unsigned i=0; i<height; i++){
			for(unsigned j=0; j<width; j++){
				cout << matrix[i][j];
			}
			cout << endl;
		}
	}
};


int main(){
	parser p;
	vector<vector<bool>> matrix = p.parse("lol_map_ascii.pbm");
	
	p.showMatrix(matrix);
	
	return 0;
}





