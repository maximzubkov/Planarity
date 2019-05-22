#include "Graph.h"
#include <iostream>
// #include <vector>
#include <iterator>
// #include <map>
using namespace std;

// Будем счиать, что подаваемый на вход граф является неориентированным
// так как ориентация графа никак не влияет на то, может ли он быть располоен на плоскости или нет
// поэтому любой приходящий граф будет рассматриваться как неориентированный 
bool Graph::isContactSubSet_ (Graph seg, Graph G1, Graph G2){
    auto seg_edges = seg.getEdges();
    auto G1_vertexes = G1.getVertexes();
    auto G2_vertexes = G2.getVertexes();
    auto faces_vertexes = this->getVertexes();
    for (const auto & edge: seg_edges){
        if (G1_vertexes.check(edge.first) && G2_vertexes.check(edge.second)){
            if (!faces_vertexes.check(edge.first))
                return false;
        }
        if (G2_vertexes.check(edge.first) && G1_vertexes.check(edge.second)){
            if (!faces_vertexes.check(edge.second))
                return false;
        }
    }
    return true;
}

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

void Graph::addEdge(const edge_t edge){
    if (std::find(edges_.begin(), edges_.end(), edge) != edges_.end()){
        std::cout << "already here";
        return;
    }
    if (edge.first + 1 > graph_matrix_.size() || edge.second + 1> graph_matrix_.size()){
        size_t size = (edge.first > edge.second ? edge.first : edge.second) + 1;
        while (size != graph_matrix_.size()){
            graph_matrix_.push_back(std::vector<size_t>(graph_matrix_.size()));
            for (auto & elem: graph_matrix_){
                elem.push_back(0);
            }
        }
    }
    graph_matrix_[edge.first][edge.second] = 1;
    edges_.push_back(edge);
    graph_list_[edge.first].push_back(edge.second);
    if (!vertexes_.check(edge.first))
        vertexes_.push_back(Vertex(edge.first));
    if (!vertexes_.check(edge.second))
        vertexes_.push_back(Vertex(edge.second));
}

void Graph::addEdgeList(const std::vector<edge_t> edges){
    for (const auto & edge: edges){
        this->addEdge(edge);
    }
}

void Graph::removeEdge(const edge_t edge){
    if (edge.first + 1 > graph_matrix_.size() || edge.second + 1> graph_matrix_.size()){
        std::cout << "unable to remove";
        return;
    }
    if (std::find(edges_.begin(), edges_.end(), edge) != edges_.end()){
        graph_matrix_[edge.first][edge.second] = 0;
        edges_.erase(std::remove(edges_.begin(), edges_.end(), edge), edges_.end());
        graph_list_[edge.first].erase(std::remove(graph_list_[edge.first].begin(), graph_list_[edge.first].end(), edge.second), graph_list_[edge.first].end());
        bool is_empty = true;
        if (std::find(graph_matrix_[edge.first].begin(), graph_matrix_[edge.first].end(), 1) == graph_matrix_[edge.first].end()){
            for (const auto & line: graph_matrix_){
                if (line[edge.first] == 1){
                    is_empty = false;
                    break;
                }
            }
            if (is_empty == true)
                vertexes_.remove(Vertex(edge.first));
        }            
        is_empty = true;
        if (std::find(graph_matrix_[edge.second].begin(), graph_matrix_[edge.second].end(), 1) == graph_matrix_[edge.second].end()){
            for (const auto & line: graph_matrix_){
                if (line[edge.second] == 1){
                    is_empty = false;
                    break;
                }
            }
            if (is_empty == true)
                vertexes_.remove(Vertex(edge.second));
        } 
    }
}

void Graph::removeEdgeList(const std::vector<edge_t> edges){
    for (const auto & edge: edges){
        this->removeEdge(edge);
    }
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
    return edges_.size();
}



std::vector<edge_t> Graph::restoreCycle_(std::vector<edge_t> cycle, size_t from, size_t to){
    if (from == to)
        return cycle;
    else
        cycle = restoreCycle_(cycle, vertexes_[from].getPi(), to);

    cycle.insert(cycle.begin(), edge_t(from, vertexes_[from].getPi()));
    cycle.insert(cycle.begin(), edge_t(vertexes_[from].getPi(), from));
    return cycle;
}

std::vector<edge_t> Graph::Cycle_visit_(size_t v, std::vector<edge_t> cycle){
    vertexes_[v].setGrey();
    std::cout << vertexes_[v].getName() << vertexes_[v].getColor() << "\n";
    for (auto & u : graph_list_[v]){
        if (vertexes_[u].getColor() == GRAY && u != v && u != vertexes_[v].getPi()){
            cycle.insert(cycle.begin(), edge_t(u, v));
            cycle.insert(cycle.begin(), edge_t(v, u));
            cycle = restoreCycle_(cycle, v, u);
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
    vertexes_[v].setBlack();

    std::cout << vertexes_[v].getName() << " IN" << vertexes_[v].getTimeIn() << " OUT" << vertexes_[v].getTimeOut()<< "\n";
    return cycle;
}

std::vector<edge_t> Graph::getCycle(){
    std::vector<edge_t> cycle;
    for (int vert = 0; vert < vertexes_.size(); ++vert){
        if (vertexes_.check(vert) && vertexes_[vert].getColor() == WHITE){
            std::cout << " efw"<< vert << "\n";
            cycle = Cycle_visit_(vert, cycle);
            if(cycle.size() != 0){
                return cycle;
            }
        }
    }
    return cycle;
}

std::vector<edge_t> Graph::Companents_visit_(size_t v, std::vector<edge_t> comp){
    vertexes_[v].setGrey();
    for (auto & u : graph_list_[v]){
        if (vertexes_[u].getColor() == WHITE && u != v){
            vertexes_[u].setPi(v);
            comp.push_back(edge_t(u,v));
            comp.push_back(edge_t(v,u));
            comp = Companents_visit_(u, comp);
        }
    }
    vertexes_[v].setBlack();
    std::cout << vertexes_[v].getName() << " IN" << vertexes_[v].getTimeIn() << " OUT" << vertexes_[v].getTimeOut()<< "\n";
    
    return comp;
}

std::vector<Graph> Graph::getCompanents(){
    std::vector<Graph> companents;
    std::vector<edge_t> tmp;
    Graph tmp_gr;
    for (int vert = 0; vert < vertexes_.size(); ++vert){
        if (vertexes_.check(vert) && vertexes_[vert].getColor() == WHITE){
            std::cout << "jump" << vert << "\n";
            tmp = Companents_visit_(vert, tmp);
            // std::cout << "comp\n";
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

std::vector<edge_t> Graph::restoreChain_(std::vector<edge_t> chain, size_t vertex){
    // std::cout << vertex << " ";
    if (vertex == vertexes_[vertex].getPi() || vertexes_[vertex].getColor() != GRAY){
        return chain;
    } else {
        chain = restoreChain_(chain, vertexes_[vertex].getPi());
    }
    chain.insert(chain.begin(), edge_t(vertex, vertexes_[vertex].getPi()));
    chain.insert(chain.begin(), edge_t(vertexes_[vertex].getPi(), vertex));
    return chain;
}

std::vector<edge_t> Graph::Chain_visit_(size_t v, std::vector<edge_t> chain, Graph G1){
    vertexes_[v].setGrey();
    auto G1_vertexes = G1.getVertexes();
    auto G1_matrix = G1.getMatrix();
    for (auto & u : graph_list_[v]){
        if (vertexes_.check(u) && vertexes_[u].getColor() == WHITE && u != v){
            vertexes_[u].setPi(v);
            // std::cout << u << " " << (std::find(G1_matrix[u].begin(), G1_matrix[u].end(), 1) != G1_matrix[u].end()) << "\n";
            if (G1_vertexes.check(u) && std::find(G1_matrix[u].begin(), G1_matrix[u].end(), 1) != G1_matrix[u].end()){
                vertexes_[u].setGrey();
                chain = restoreChain_(chain, u);
                // std::cout << "chain " << u << " " << v;
                for (const auto & elem : chain){
                    std::cout << elem.first << " " << elem.second << "\n";
                }
                std::cout << std::endl;
                return chain;
            }
            else
                chain = Chain_visit_(u, chain, G1);
        }
    }
    vertexes_[v].setBlack();
    std::cout << vertexes_[v].getName() << " IN" << vertexes_[v].getTimeIn() << " OUT" << vertexes_[v].getTimeOut()<< "\n";
    
    return chain;
}

std::vector<edge_t> Graph::getChain(Graph G1){
    auto G1_vertexes = G1.getVertexes();
    auto G1_matrix = G1.getMatrix();
    std::vector<edge_t> chain;
    for (int vert = 0; vert < vertexes_.size(); ++vert){
        if (vertexes_.check(vert) && vertexes_[vert].getColor() == WHITE && G1_vertexes.check(vert)
            && std::find(G1_matrix[vert].begin(), G1_matrix[vert].end(), 1) != G1_matrix[vert].end()){
            vertexes_[vert].setPi(vert);
            std::cout << "chhhain" << vert << "\n";
            return Chain_visit_(vert, chain, G1);
            // for (const auto& elem: tmp){
            //     std::cout << elem.first << " " << elem.second << std::endl;
            // }
        }
    }
    return chain;
}

// std::vector <std::vector <Vertex>> getFaces(){

// }

std::vector <edge_t> Graph::split_(const std::vector<edge_t> split_chain){
    std::vector<edge_t> res;
    std::cout << std::endl;
    for (const auto & edge: edges_){
        if (std::find(split_chain.begin(), split_chain.end(), edge) == split_chain.end()){
            std::cout << edge.first << " " << edge.second << "\n";
            res.push_back(edge);
        }
    }
    return res;
}

std::vector <Graph> Graph::getSegments_(Graph G1, Graph G2){
    auto G1_vertexes = G1.getVertexes();
    auto G1_edges = G1.getEdges();
    auto G2_edges = G2.getEdges();
    auto all_edges = this->getEdges();
    
    auto segments = G2.getCompanents();
    VertexList seg_vertexes; 
    std::vector <edge_t> seg_edges;
    for (auto & seg: segments){
        seg_vertexes = seg.getVertexes();
        seg_edges = seg.getEdges();
        for (const auto & edge: G2_edges){
            if (std::find(seg_edges.begin(), seg_edges.end(), edge) == seg_edges.end()){
                if (seg_vertexes.check(edge.first) && G1_vertexes.check(edge.second)){
                    if (std::find(G1_edges.begin(), G1_edges.end(), edge) == G1_edges.end()){
                        seg.addEdge(edge);
                    }
                }
                if (seg_vertexes.check(edge.second) && G1_vertexes.check(edge.first)){
                    if (std::find(G1_edges.begin(), G1_edges.end(), edge) == G1_edges.end()){
                        seg.addEdge(edge);
                    }
                }
            }
        }
    }
    std::cout << "seg check;";
    for (auto & seg: segments){
        for (auto & elem: seg.getEdges()){
            std::cout << elem.first << " " << elem.second << "    ";
        }
        std::cout << "\n\n";
    }

    std::vector <edge_t> tmp;
    // Плохо, переписать
    bool already_used = false;
    for (const auto & edge: G2_edges){
        if (G1_vertexes.check(edge.first) && G1_vertexes.check(edge.second) && std::find(G1_edges.begin(), G1_edges.end(), edge) == G1_edges.end()){
            for (const auto & seg: segments){
                seg_edges = seg.getEdges();
                if (std::find(seg_edges.begin(), seg_edges.end(), edge) != seg_edges.end()){
                    already_used = true;
                    break;
                }
            }
            if (!already_used){
                tmp.clear();
                tmp.push_back(edge);
                tmp.push_back(edge_t(edge.second, edge.first));
                for(const auto & elem: tmp){
                    std::cout << elem.first << " " << elem.second << "  ";
                }
                std::cout << std::endl;
                if (std::find(segments.begin(), segments.end(), Graph(tmp)) == segments.end())
                   segments.push_back(Graph(tmp)); 
            }
        }
    }

    std::cout << "segs!\n";

    for (auto & seg: segments){
        std::cout << "\n" << seg << "\n";
    }  
    return segments;

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
    std::cout << "G1\n" << G1 << "\n";
    G1.MakeUndirect();
    std::vector <edge_t> other_edges;
    for (const auto & edge: edges_){
        if (std::find(cycle.begin(), cycle.end(), edge) == cycle.end()){
            other_edges.push_back(edge);
            // other_edges.push_back(edge_t(edge.second, edge.first));
        }

    }

    Graph G2(other_edges);
    std::cout << "G2\n\n" << G2;
    segments_ = this->getSegments_(G1, G2);
    faces_.push_back(G1);
    std::vector <Vertex> seg_vert;
    Graph min_seg;
    size_t count = 0;
    size_t min = INT_MAX;
    std::vector <edge_t> chain;
    std::vector <edge_t> edges_of_new_segs;
    int x = 0;
    while (G2.size() != 0 && x < 3){
        for (auto & seg: segments_){
            for (auto & face: faces_){
                if (face.isContactSubSet_(seg, G1, G2)){
                    count++;
                }
            }
            if (count == 0)
                std::cout << "No planarity";

            if(count < min){
                min = count;
                min_seg = seg;
            }
        }
        std::cout << "min segg "<< min_seg << "\n";
        chain = min_seg.getChain(G1);
        std::cout << G2.size();
        std::cout << "G2 check;";
        for (auto & elem: G2.getEdges()){
            std::cout << elem.first << " " << elem.second << "  ";
        }
        std::cout << "\n\n";
        G2.removeEdgeList(chain);
        for (auto & elem: G2.getEdges()){
            std::cout << elem.first << " " << elem.second << "  ";
        }
        // for (const auto & elem: G2.getVertexes()){
        //     std::cout << elem.getName() << " ";
        // }
        std::cout << "\n";
        std::cout << G2;
        edges_of_new_segs =  min_seg.split_(chain);
        G1.addEdgeList(chain);
        G2.addEdgeList(edges_of_new_segs);
        std::cout << G1;
        segments_ = this->getSegments_(G1, G2);
        for (const auto & elem: segments_){
            std::cout << elem << "\n\n";
        }
        std::cout << G2.size();
        x++;
        // break;
        // std::remove(segments_.begin(), segments_.end(), min_seg);
    }
    // for (const auto & elem: segments_insert_count_){
    //     std::cout << elem << " ";
    // }   
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

std::ostream& operator<< (std::ostream &ostr, const Graph &graph){
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