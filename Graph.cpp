#include "Graph.h"
#include <iostream>
// #include <vector>
#include <iterator>
// #include <map>
using namespace std;

// Будем счиать, что подаваемый на вход граф является неориентированным
// так как ориентация графа никак не влияет на то, может ли он быть располоен на плоскости или нет
// поэтому любой приходящий граф будет рассматриваться как неориентированный 
Graph::Graph(size_t n_vertex, std::vector< std::vector<size_t> >& matrix) : 
    graph_size_(n_vertex), 
    graph_matrix_(matrix) 
{
    // Конструктор класса, преобразующй
    std::cout << "Graph object being created with max nodes size:" << n_vertex << "\n\n";
    uint64_t code = 0;
    for (size_t line = 0; line != graph_matrix_.size(); ++line){
        code = 0;
        for (size_t elem = 0; elem != graph_matrix_[line].size(); ++elem){
            code += (static_cast<uint64_t>(graph_matrix_[line][elem]) << (graph_size_ - line - 1));
            if (graph_matrix_[line][elem] == 1){
                graph_list_[line].push_back(elem);
                graph_list_[elem].push_back(line);
                edges_.push_back(std::make_pair(line, elem));
                edges_.push_back(std::make_pair(elem, line));
            }
        } 
        vertexes_.push_back(Vertex(line));
        graph_bit_code_.push_back(code);
    }
}

Graph::Graph(size_t n_vertex, std::vector< std::pair<size_t, size_t> >& edges):
    graph_size_(n_vertex), 
    graph_matrix_(std::vector<std::vector<size_t>>(n_vertex, std::vector<size_t>(n_vertex, 0))),
    edges_(edges)
{
    // Конструктор класса, преобразующй
    std::cout << "Graph object being created with max nodes size:" << n_vertex << "\n\n";
    for (const auto & elem : edges){
        graph_matrix_[elem.first][elem.second] = 1;
        graph_matrix_[elem.second][elem.first] = 1;
    }
    for (int u = 0; u < graph_matrix_.size(); ++u){
        for (int v = 0; v < graph_matrix_.size(); ++v){
            if (graph_matrix_[u][v] == 1)
                graph_list_[u].push_back(v);
        }
    }
    for (int v = 0; v < n_vertex; v++){
        vertexes_.push_back(Vertex(v));
    }
}

size_t Graph::size() const {
    return graph_size_;
}

void Graph::GetCycle_(size_t from, size_t to){
    cycle_.insert(cycle_.begin(), from);
    if (from == to){
        return;
    } else {
        GetCycle_(vertexes_[from].getPi(), to);
    }
    return;
}

size_t Graph::DFS_visit_(size_t v, size_t dfs_timer){
    // v.setTimeIn(dfs_timer);
    // v.setGrey();
    vertexes_[v].setTimeIn(dfs_timer);
    vertexes_[v].setGrey();
    dfs_timer++;
    for (auto & u : graph_list_[v]){
        if (vertexes_[u].getColor() == GRAY && u != v && u != vertexes_[v].getPi()){
            // vertexes_[u].setPi(v);
            GetCycle_(v, u);
            std::cout << "cycle ";
            for (const auto & elem : cycle_){
                std::cout << elem;
            }
            std::cout << std::endl;
        }
        if (vertexes_[u].getColor() == WHITE && u != v){
            vertexes_[u].setPi(v);
            dfs_timer = DFS_visit_(u, dfs_timer);
        }
    }
    vertexes_[v].setTimeOut(dfs_timer);
    vertexes_[v].setBlack();
    dfs_timer++;
    // v.setTimeOut(dfs_timer);
    // v.setBlack();
    std::cout << vertexes_[v].getName() << " IN" << vertexes_[v].getTimeIn() << " OUT" << vertexes_[v].getTimeOut()<< "\n";
    return dfs_timer;
}

void Graph::DFS(){
    size_t dfs_timer = 0;
    for (int vert = 0; vert < vertexes_.size(); vert++){
        if (vertexes_[vert].getColor() == WHITE){
            dfs_timer = DFS_visit_(vert, dfs_timer);
        }
    }
}

std::vector <std::set <std::pair<size_t, size_t> > > Graph::SegmentsFind_(){
    std::vector <std::set <std::pair<size_t, size_t> > > S;
    for(const auto & v : graph_list_){
        if (vertexes_[v.first].getStatus() == G2){
            for(const auto & u : v.second){
                if (vertexes_[u].getStatus() == G2){
                    S.push_back(std::set<std::pair<size_t,size_t>> {std::make_pair (u,v.first)});
                    S.push_back(std::set<std::pair<size_t,size_t>> {std::make_pair (v.first, u)});
                }
            }
        }
    }
    for (const auto & v : graph_list_){
        if (vertexes_[v.first].getStatus() == G1){
            break;
        }
    }
    return S;
}

std::vector< std::set<int> > Graph::Faces(){
    std::vector< std::set<int> > res;
    DFS();
    for (auto & v: cycle_){
        vertexes_[v].changeStatus(G2);
    }
    // Сформируем множество сегментов, если оно окажется пусто, то 
    // алгоритм завершен, в противном случае:
    //      1. Для каждого сегмента S найдем множество Г(S), то есть грани вмещающие в себя сегмент S
    //         если найдется такой сегмент, что |Г(S)| = 0, то граф планарен
    //      2. Выберем сегмент с минимальным числом вмещающих граней
    //      3. Выберем одну из подходящих граней
    //      4. Выберем цепь между двумя контактными вершинами и уложим ее в выбранной грани. Если теперь
    //         множество сегментов не пусто, то перейдем к пункту 1.
    return res;
}

std::vector < std::vector <size_t> > Graph::getMatrix() const{
    return graph_matrix_;
}

std::map <size_t, std::vector<size_t> > Graph::getList() const{
    return graph_list_;
}

std::vector <std::pair<size_t, size_t> > Graph::getEdges() const{
    return edges_;
}

// std::vector <uint64_t> Graph::getBiteCodes() const {
//     return graph_bit_code_;
// }

std::ostream& operator << (std::ostream &ostr, const Graph &graph){
    auto matrix = graph.getMatrix();
    // std::map <Vertex, std::vector<Vertex> > list = graph.getList();
    for (const auto& line: matrix){
        for (const auto& elem: line){
            ostr << elem << " ";
        }
        ostr << std::endl;
    }
    return ostr;
}