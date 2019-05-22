#pragma once
#include "Vertex.h"
#include <vector>
#include <map>
#include <set>
// #define INT_MAX 1000000

typedef std::pair<size_t, size_t> edge_t;

class Graph{
    public:
        Graph(std::vector <std::vector <size_t>>& matrix);
        Graph(std::vector <edge_t>& list); 
        Graph() = default;
        size_t size() const;
        std::vector <std::vector <size_t>> getMatrix() const;
        std::map <size_t, std::vector <size_t>> getList() const;
        std::vector <edge_t> getEdges() const;
        VertexList getVertexes() const;
        void MakeUndirect();
        std::vector <edge_t> getCycle();
        std::vector <Graph> getCompanents();
        std::vector <edge_t> getChain(Graph G1);
        std::pair <Graph, Graph> getFaces(Graph cur_face, std::vector<edge_t> chain);
        std::vector <edge_t> findPath(size_t start_vertex, size_t finish_vertex, std::vector<edge_t> path);
        void Gamma();
        void addEdge(const edge_t edge);   
        void removeEdge(const edge_t edge);   
        void addEdgeList(const std::vector<edge_t> edges);
        void removeEdgeList(const std::vector<edge_t> edges);
        // std::vector <uint64_t> getBiteCodes() const;
        friend std::ostream& operator << (std::ostream &ostr, const Graph &graph);
        friend bool operator== (const Graph &gr1, const Graph &gr2);

    private:
        size_t graph_size_;
        size_t dfs_timer_;
        std::vector < std::vector <size_t> > graph_matrix_;
        std::map <size_t, std::vector<size_t> > graph_list_;
        std::vector <uint64_t> graph_bit_code_;
        VertexList vertexes_;
        std::vector <edge_t> edges_;
        std::vector <edge_t> cycle_;
        std::vector <Graph> segments_;
        std::vector <size_t> segments_insert_count_;
        std::vector <Graph> faces_;
        std::vector <Graph> getSegments_(Graph G1, Graph G2);
        std::vector <edge_t> Cycle_visit_(size_t v, std::vector<edge_t> comp);
        std::vector <edge_t> Chain_visit_(size_t v, std::vector<edge_t> chain, Graph G1);
        std::vector <edge_t> Companents_visit_(size_t v, std::vector<edge_t> comp);
        std::vector <edge_t> restoreCycle_(std::vector<edge_t> cycle, size_t from, size_t to);
        std::vector <edge_t> restoreChain_(std::vector<edge_t> chain, size_t vertex);
        std::vector <edge_t> split_(const std::vector<edge_t> split_chain);
        bool isContactSubSet_ (Graph seg, Graph G1, Graph G2);
        bool pred(const edge_t &a, edge_t &b);
};

