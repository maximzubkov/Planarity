#pragma once
#include "Vertex.h"
#include <vector>
#include <map>
#include <set>


class Graph{
    public:
        Graph(size_t n_vertex, std::vector< std::vector<size_t> >& matrix);
        Graph(size_t n_vertex, std::vector< std::pair<size_t, size_t> >& list); 
        // Graph(std::size_t n_vertex);
        size_t size() const;
        std::vector < std::vector <size_t> > getMatrix() const;
        std::map <size_t, std::vector<size_t> > getList() const;
        std::vector <std::pair<size_t, size_t>>  getEdges() const;
        bool isAcyclic() const;
        void DFS();
        std::vector< std::set<int> > Faces();
        std::vector <std::set <std::pair<size_t, size_t> > > SegmentsFind_();
        // std::vector <uint64_t> getBiteCodes() const;
        friend std::ostream& operator << (std::ostream &ostr, const Graph &graph);
    private:
        std::size_t graph_size_;
        std::vector < std::vector <size_t> > graph_matrix_;
        std::map <size_t, std::vector<size_t> > graph_list_;
        std::vector <uint64_t> graph_bit_code_;
        std::vector <Vertex> vertexes_;
        std::vector <std::pair<size_t, size_t>> edges_;
        std::vector <size_t> cycle_;
        size_t DFS_visit_(size_t u, size_t dfs_timer);
        void GetCycle_(size_t from, size_t to);
};

