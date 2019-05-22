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
    if (seg_edges.empty())
        return false;
    auto G1_vertexes = G1.getVertexes();
    auto G2_vertexes = G2.getVertexes();
    auto faces_vertexes = getVertexes();
    if (seg_edges.size() == 2){
        for (const auto & edge: seg_edges){
            if (G1_vertexes.check(edge.first) && G1_vertexes.check(edge.second)){
                if (!faces_vertexes.check(edge.first) || !faces_vertexes.check(edge.second))
                    return false;
            }
        }
    }
    for (const auto & edge: seg_edges){
        if (G1_vertexes.check(edge.first) && G2_vertexes.check(edge.second)){
            if (!faces_vertexes.check(edge.first) && !(G2_vertexes.check(edge.first) && G1_vertexes.check(edge.second) &&
                faces_vertexes.check(edge.second))){
                return false;
            }
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
    for (const auto & edge : edges_)
        graph_matrix_[edge.first][edge.second] = 1;

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
        addEdge(edge);
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
        removeEdge(edge);
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

    cycle.insert(cycle.begin(), edge_t(vertexes_[from].getPi(), from));
    return cycle;
}

std::vector<edge_t> Graph::Cycle_visit_(size_t v, std::vector<edge_t> cycle){
    vertexes_[v].setGrey();
    for (auto & u : graph_list_[v]){
        if (vertexes_[u].getColor() == GRAY && u != v && u != vertexes_[v].getPi()){
            cycle.insert(cycle.begin(), edge_t(v, u));
            cycle = restoreCycle_(cycle, v, u);
            return cycle;
        }
        if (vertexes_[u].getColor() == WHITE && u != v){
            vertexes_[u].setPi(v);
            return Cycle_visit_(u, cycle);
        }
    }
    vertexes_[v].setBlack();
    return cycle;
}

std::vector<edge_t> Graph::getCycle(){
    std::vector<edge_t> cycle;
    for (int vert = 0; vert < vertexes_.max(); ++vert){
        if (vertexes_.check(vert) && vertexes_[vert].getColor() == WHITE){
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
    return comp;
}

std::vector<Graph> Graph::getCompanents(){
    std::vector<Graph> companents;
    std::vector<edge_t> tmp;
    Graph tmp_gr;
    for (int vert = 0; vert < vertexes_.max(); ++vert){
        if (vertexes_.check(vert) && vertexes_[vert].getColor() == WHITE){
            tmp = Companents_visit_(vert, tmp);
            tmp_gr = Graph(tmp);
            companents.push_back(tmp_gr);
            tmp.clear();
        }
    }
    return companents;
}

std::vector<edge_t> Graph::restoreChain_(std::vector<edge_t> chain, size_t vertex){
    if (vertex == vertexes_[vertex].getPi()){
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
    if (graph_list_[v].empty()){
        chain = restoreChain_(chain, v);
        return chain;
    }
    for (auto & u : graph_list_[v]){
        if (vertexes_.check(u) && vertexes_[u].getColor() == WHITE && u != v){
            vertexes_[u].setPi(v);
            if (G1_vertexes.check(u)){
                vertexes_[u].setGrey();
                chain = restoreChain_(chain, u);
                return chain;
            }
            else
                chain = Chain_visit_(u, chain, G1);
        }
    }
    vertexes_[v].setBlack();    
    return chain;
}

std::vector<edge_t> Graph::getChain(Graph G1){
    auto G1_vertexes = G1.getVertexes();
    auto G1_matrix = G1.getMatrix();
    std::vector<edge_t> chain;
    for (int vert = 0; vert < vertexes_.max(); ++vert){
        if (vertexes_.check(vert) && vertexes_[vert].getColor() == WHITE && G1_vertexes.check(vert)){
            vertexes_[vert].setPi(vert);
            return Chain_visit_(vert, chain, G1);
        }
    }
    return chain;
}


std::vector <edge_t> Graph::split_(const std::vector<edge_t> split_chain){
    std::vector<edge_t> res;
    for (const auto & edge: edges_){
        if (std::find(split_chain.begin(), split_chain.end(), edge) == split_chain.end()){
            res.push_back(edge);
        }
    }
    return res;
}

std::vector <Graph> Graph::getSegments_(Graph G1, Graph G2){
    auto G1_vertexes = G1.getVertexes();
    auto G1_edges = G1.getEdges();
    auto G2_edges = G2.getEdges();
    auto all_edges = getEdges();
    
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
                if (std::find(segments.begin(), segments.end(), Graph(tmp)) == segments.end())
                   segments.push_back(Graph(tmp)); 
            }
        }
    }
    return segments;

}

std::vector<edge_t> Graph::findPath(size_t start_vertex, size_t finish_vertex, std::vector<edge_t> path){
    size_t next = 0;
    if (start_vertex == finish_vertex){
        return path;
    } else {
        for (const auto & edge: edges_){
            if (edge.first == start_vertex){
                next = edge.second;
                break;
            }
        }
        path = findPath(next, finish_vertex, path);
    }
    auto edge = edge_t(start_vertex, next);
    path.push_back(edge);
    return path;
}

std::pair<Graph, Graph> Graph::getFaces(Graph cur_face, std::vector<edge_t> chain){
    std::vector<edge_t> face1_edges;
    std::vector<edge_t> face2_edges;
    Graph face1;
    Graph face2;
    auto face_vertxes = cur_face.getVertexes();
    int iter = 0;
    for (const auto & elem: chain){
        if (iter % 2 == 0)
            face1_edges.push_back(elem);
        if (iter % 2 == 1)
            face2_edges.push_back(elem);  
        iter++;  
    }
    size_t start_vertex;
    size_t finish_vertex;
    if (face1_edges.size() > 1){
        start_vertex = face1_edges.front().second;
        finish_vertex = face1_edges.back().first;
    } else {
        start_vertex = face1_edges[0].second;
        finish_vertex = face1_edges[0].first;
    }

    if (face_vertxes.check(start_vertex) && face_vertxes.check(finish_vertex)){
        face1_edges = cur_face.findPath(start_vertex, finish_vertex, face1_edges);
        for (const auto & edge: cur_face.getEdges()){
            if (std::find(face1_edges.begin(), face1_edges.end(), edge) == face1_edges.end())
                face2_edges.push_back(edge);
        }
        face1 = Graph(face1_edges);
        face2 = Graph(face2_edges);
    }
    if (face_vertxes.check(start_vertex) && !face_vertxes.check(finish_vertex)){
        cur_face.addEdgeList(chain);
        face1 = cur_face;
        face2 = Graph();
    }
    return std::pair<Graph, Graph>(face1, face2);
}

void Graph::Gamma(){
    MakeUndirect();
    std::vector<edge_t> cycle = getCycle();
    if (cycle.size() == 0){
        std::cout << "forest" << std::endl;
        return;
    }

    Graph G1(cycle);
    G1.MakeUndirect();
    std::vector<edge_t> face1_edges;
    std::vector<edge_t> face2_edges;
    for (const auto & elem: cycle){
        face1_edges.push_back(elem);
        face2_edges.push_back(edge_t(elem.second, elem.first));     
    }
    faces_.push_back(Graph(face1_edges));
    faces_.push_back(Graph(face2_edges));
    std::vector <edge_t> other_edges;
    for (const auto & edge: edges_){
        if (std::find(face1_edges.begin(), face1_edges.end(), edge) == face1_edges.end() && 
            std::find(face2_edges.begin(), face2_edges.end(), edge) == face2_edges.end()){
            other_edges.push_back(edge);
        }

    }

    Graph G2(other_edges);
    segments_ = getSegments_(G1, G2);
    std::vector <Vertex> seg_vert;
    Graph min_seg;
    size_t count = 0;
    size_t min = INT_MAX;
    std::vector <edge_t> chain;
    std::vector <edge_t> edges_of_new_segs;
    Graph cur_face;
    std::pair<Graph, Graph> new_faces;
    while (G2.size() != 0){
        min = INT_MAX;
        for (auto & seg: segments_){
            count = 0;
            for (auto & face: faces_){
                if (face.isContactSubSet_(seg, G1, G2) == true){
                    count++;
                }
            }

            if(count < min){
                min = count;
                min_seg = seg;
            }
        }
        chain = min_seg.getChain(G1);
        for (auto & face: faces_){
            if (face.isContactSubSet_(min_seg, G1, G2)){
                cur_face = face;
                break;
            }
        }

        new_faces = getFaces(cur_face, chain);
        if (new_faces.second.size() == 0){
            faces_.push_back(new_faces.first);
        } else {
            faces_.push_back(new_faces.first);
            faces_.push_back(new_faces.second);
        }   
        faces_.erase(std::remove(faces_.begin(), faces_.end(), cur_face), faces_.end());
        std::cout << "faces::";
        for (const auto &face: faces_){
            std::cout << face << "\n";
        }
        G2.removeEdgeList(chain);
        edges_of_new_segs =  min_seg.split_(chain);
        G1.addEdgeList(chain);
        G2.addEdgeList(edges_of_new_segs);
        segments_.clear();
        segments_ = getSegments_(G1, G2);
    }
    std::cout << "faces::";
    for (const auto &face: faces_){
        std::cout << face << "\n";
    }
    return;
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