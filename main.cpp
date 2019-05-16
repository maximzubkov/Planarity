#include "Graph.h"
#include "Vertex.h"
#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <sstream>

std::vector< std::vector<size_t> > get_graph_from_file(std::string dir){
    // Открывает файл graph.txt, преобразует данные из него в матрицу инцедентности
    std::ifstream in;
    in.open(dir);
    if (!in.is_open()){
        std::cout << "invalid file";
        exit(0);
    }
    std::string line;
    std::string tmp_str;
    std::vector <size_t> tmp_vect;
    std::vector < std::vector <size_t> > res;

    while(std::getline(in, line)){
        std::istringstream ist(line);
        while (ist >> tmp_str){
            tmp_vect.push_back(std::stoi(tmp_str));
        }
        res.push_back(tmp_vect);
        tmp_vect.clear();
    }
    in.close();
    return res;
}

// /Users/MaximZubkov/Desktop/Math/AMC/bonus2/graph.txt это я для себя

int main(int argc, char * argv[]){
	auto matrix = get_graph_from_file(argv[1]);
	Graph graph(matrix.size(), matrix);
	std::cout << graph;
	auto m = graph.getEdges();
    Graph graph2(matrix.size(), m);

	// for (auto& s : m){
	// 	std::cout << s.first << " "; 
	// 	for (auto & elem: s.second){
	//     	std::cout << elem << " ";
	// 	}
	// 	std::cout << std::endl;
	// } 
	auto v = graph2.getMatrix();
	for (auto& line : v){
		for (auto& elem : line){
	    	std::cout << elem << " ";
		}
		std::cout << std::endl;
	} 
    graph.DFS();
	graph2.DFS();
    std::vector <std::set <std::pair<int, int> > > d;
    // auto pair = std::set<std::pair<int,int>> {std::make_pair (1,2)};
    // d.push_back(std::set<std::pair<int,int>> (pair));
	return 0;
}