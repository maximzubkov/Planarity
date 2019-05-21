#include "Graph.h"
#include <iostream>
// #include <vector>
#include <iterator>
// #include <map>
using namespace std;

// Будем счиать, что подаваемый на вход граф является неориентированным
// так как ориентация графа никак не влияет на то, может ли он быть располоен на плоскости или нет
// поэтому любой приходящий граф будет рассматриваться как неориентированный 
Graph::Graph(std::vector< std::vector<size_t> >& matrix) : 
    graph_size_(matrix.size()), 
    graph_matrix_(matrix) 
{
    // Конструктор класса, преобразующй
    std::cout << "Graph object being created with max nodes size:" << matrix.size() << "\n\n";
    uint64_t code = 0;
    for (size_t line = 0; line != graph_matrix_.size(); ++line){
        code = 0;
        for (size_t elem = 0; elem != graph_matrix_[line].size(); ++elem){
            code += (static_cast<uint64_t>(graph_matrix_[line][elem]) << (graph_size_ - line - 1));
            if (graph_matrix_[line][elem] == 1){
                graph_list_[line].push_back(elem);
                edges_.push_back(edge_t(line, elem));
            }
        } 
        vertexes_.push_back(Vertex(line));
        graph_bit_code_.push_back(code);
    }
}

Graph::Graph(std::vector<edge_t>& edges):
    edges_(edges)
{
    // Конструктор класса, преобразующй
    size_t max_vertex_index = 0;
    for (const auto & edge: edges_){
        if (edge.first > max_vertex_index)
            max_vertex_index = edge.first;
        if (edge.second > max_vertex_index)
            max_vertex_index = edge.second;
    }
    graph_size_ = max_vertex_index + 1;
    graph_matrix_ = std::vector<std::vector<size_t>>(graph_size_, std::vector<size_t>(graph_size_, 0));
    std::cout << "Graph object being created with max nodes size:" << graph_size_ << "\n\n";
    for (const auto & edge : edges_){
        // std::cout << edge.first << " " << edge.second;
        graph_matrix_[edge.first][edge.second] = 1;
        // graph_matrix_[elem.second][elem.first] = 1;
    }
    for (int u = 0; u < graph_matrix_.size(); ++u){
        for (int v = 0; v < graph_matrix_.size(); ++v){
            if (graph_matrix_[u][v] == 1)
                graph_list_[u].push_back(v);
        }
    }
    std:cout << " " << graph_size_ << std::endl;
    for (const auto & edge: edges_){
        if (!vertexes_.check(edge.first))
            vertexes_.push_back(Vertex(edge.first));
        if (!vertexes_.check(edge.second))
            vertexes_.push_back(Vertex(edge.second));
    }
}

void Graph::addEdge(edge_t edge){
    graph_matrix_[edge.first][edge.second] = 1;
    edges_.push_back(edge);
    graph_list_[edge.first].push_back(edge.second);
    if (!vertexes_.check(edge.first))
        vertexes_.push_back(Vertex(edge.first));
    if (!vertexes_.check(edge.second))
        vertexes_.push_back(Vertex(edge.second));
}

void Graph::MakeUndirect(){
    for (int u = 0; u < graph_matrix_.size(); ++u){
        for (int v = 0; v < graph_matrix_.size(); ++v){
            if (graph_matrix_[u][v] == 1){
                graph_list_[v].push_back(u);
                edges_.push_back(edge_t(v, u));
            }
        }
    }
    for (int u = 0; u < graph_matrix_.size(); ++u){
        for (int v = 0; v < graph_matrix_.size(); ++v){
            if (graph_matrix_[u][v] == 1)
                 graph_matrix_[v][u] = 1;
        }
    }
}

size_t Graph::size() const {
    return graph_size_;
}

std::vector<edge_t> Graph::Get_Cycle_(std::vector<edge_t> cycle, size_t from, size_t to){
    if (from == to)
        return cycle;
    else
        cycle = Get_Cycle_(cycle, vertexes_[from].getPi(), to);

    cycle.insert(cycle.begin(), edge_t(from, vertexes_[from].getPi()));
    cycle.insert(cycle.begin(), edge_t(vertexes_[from].getPi(), from));
    return cycle;
}

std::vector<edge_t> Graph::Cycle_visit_(size_t v, std::vector<edge_t> cycle){
    vertexes_[v].setTimeIn(dfs_timer_);
    vertexes_[v].setGrey();
    std::cout << vertexes_[v].getName() << vertexes_[v].getColor() << "\n";

    dfs_timer_++;
    for (auto & u : graph_list_[v]){
        if (vertexes_[u].getColor() == GRAY && u != v && u != vertexes_[v].getPi()){
            cycle.insert(cycle.begin(), edge_t(u, v));
            cycle.insert(cycle.begin(), edge_t(v, u));
            cycle = Get_Cycle_(cycle, v, u);
            std::cout << "cycle " << u << " " << v;
            for (const auto & elem : cycle){
                std::cout << elem.first << " " << elem.second << "\n";
            }
            std::cout << std::endl;
            return cycle;
        }
        if (vertexes_[u].getColor() == WHITE && u != v){
            vertexes_[u].setPi(v);
            return Cycle_visit_(u, cycle);
        }
    }
    vertexes_[v].setTimeOut(dfs_timer_);
    vertexes_[v].setBlack();
    dfs_timer_++;

    std::cout << vertexes_[v].getName() << " IN" << vertexes_[v].getTimeIn() << " OUT" << vertexes_[v].getTimeOut()<< "\n";
    return cycle;
}

std::vector<edge_t> Graph::getCycle(){
    dfs_timer_ = 0;
    std::vector<edge_t> cycle;
    for (int vert = 0; vert < vertexes_.size(); ++vert){
        if (vertexes_[vert].getColor() == WHITE){
            std::cout << " efw"<<vert << "\n";
            cycle = Cycle_visit_(vert, cycle);
            if(cycle.size() != 0){
                return cycle;
            }
        }
    }
    return cycle;
}

std::vector<edge_t> Graph::Companents_visit_(size_t v, std::vector<edge_t> comp){
    vertexes_[v].setTimeIn(dfs_timer_);
    vertexes_[v].setGrey();
    dfs_timer_++;
    for (auto & u : graph_list_[v]){
        if (vertexes_[u].getColor() == WHITE && u != v){
            vertexes_[u].setPi(v);
            comp.push_back(edge_t(u,v));
            comp.push_back(edge_t(v,u));
            comp = Companents_visit_(u, comp);
        }
    }
    vertexes_[v].setTimeOut(dfs_timer_);
    vertexes_[v].setBlack();
    dfs_timer_++;
    std::cout << vertexes_[v].getName() << " IN" << vertexes_[v].getTimeIn() << " OUT" << vertexes_[v].getTimeOut()<< "\n";
    
    return comp;
}

std::vector<Graph> Graph::getCompanents(){
    dfs_timer_ = 0;
    std::vector<Graph> companents;
    std::vector<edge_t> tmp;
    Graph tmp_gr;
    for (int vert = 0; vert < vertexes_.size(); vert++){
        if (vertexes_[vert].getColor() == WHITE){
            std::cout << "jump" << vert << "\n";
            tmp = Companents_visit_(vert, tmp);
            // for (const auto& elem: tmp){
            //     std::cout << elem.first << " " << elem.second << std::endl;
            // }
            tmp_gr = Graph(tmp);
            companents.push_back(tmp_gr);
            tmp.clear();
        }
    }
    return companents;
}



std::vector< std::set<int> > Graph::Gamma(){
    std::vector< std::set<int> > res;
    MakeUndirect();
    std::cout << *this;
    std::vector<edge_t> cycle = getCycle();
    if (cycle.size() == 0){
        std::cout << "forst" << std::endl;
        return std::vector< std::set<int> >();
    }

    Graph G1(cycle);
    G1.MakeUndirect();

    std::vector <edge_t> other_edges;
    for (const auto & edge: edges_){
        if (std::find(cycle.begin(), cycle.end(), edge) == cycle.end()){
            other_edges.push_back(edge);
            other_edges.push_back(edge_t(edge.second, edge.first));
        }

    }

    Graph G2(other_edges);
    std::cout << "G2\n\n" << G2;
    auto G1_vertexes = G1.getVertexes();
    auto G2_edges = G2.getEdges();
    auto all_edges = this->getEdges();
    segments_ = G2.getCompanents();
    VertexList seg_vertexes;

    for (auto & seg: segments_){
        seg_vertexes = seg.getVertexes();
        for (const auto & edge: all_edges){
            if ((seg_vertexes.check(edge.first) && G1_vertexes.check(edge.second)) || 
                (seg_vertexes.check(edge.second) && G1_vertexes.check(edge.first)))
                seg.addEdge(edge);
        }
    }   

    std::vector <edge_t> tmp;
    for (const auto & edge: G2_edges){
        if (G1_vertexes.check(edge.first) && G1_vertexes.check(edge.second)){
           tmp.clear();
           tmp.push_back(edge);
           tmp.push_back(edge_t(edge.second, edge.first));
           for(const auto & elem: tmp){
                std::cout << elem.first << " " << elem.second << "  ";
           }
           std::cout << std::endl;
           if (std::find(segments_.begin(), segments_.end(), Graph(tmp)) == segments_.end())
               segments_.push_back(Graph(tmp)); 
        }
    }

    for (auto & seg: segments_){
        std::cout << "\n" << seg << "\n";
    }   
    // for (auto & v: cycle_){
    //     vertexes_[v].changeStatus(G2);
    // }
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

std::vector <edge_t> Graph::getEdges() const{
    return edges_;
}

VertexList Graph::getVertexes() const{
    return vertexes_;
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

bool operator== (const Graph &gr1, const Graph &gr2){
    auto gr1_edges = gr1.getEdges();
    auto gr2_edges = gr2.getEdges();
    bool equal_found = false;
    if (gr1_edges.size() != gr2_edges.size())
        return false;
    else {
        for (const auto elem1: gr1_edges){
            equal_found = false;
            for (const auto elem2: gr2_edges){
                if (elem1 == elem2){
                    equal_found = true;
                    break;
                }
            }
            if (!equal_found)
                return false;
        }
        return true;
    }
}

// std::vector <std::set <std::pair<size_t, size_t> > > Graph::SegmentsFind_(){
//     std::vector <std::set <std::pair<size_t, size_t> > > S;
//     for(const auto & v : graph_list_){
//         if (vertexes_[v.first].getStatus() == G2){
//             for(const auto & u : v.second){
//                 if (vertexes_[u].getStatus() == G2){
//                     S.push_back(std::set<std::pair<size_t,size_t>> {std::make_pair (u,v.first)});
//                     S.push_back(std::set<std::pair<size_t,size_t>> {std::make_pair (v.first, u)});
//                 }
//             }
//         }
//     }
//     for (const auto & v : graph_list_){
//         if (vertexes_[v.first].getStatus() == G1){
//             break;
//         }
//     }
//     return S;
// }