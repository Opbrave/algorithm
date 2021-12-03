// A simple implementation of Kahn algorithm for topological order
#include<iostream>
#include<map>
#include<vector>
#include<set>

// A simple struct to express graph
using OutDegreeVec = std::vector<int>;
using InDegreeVec = std::vector<int>;
using Edges = std::pair<OutDegreeVec, InDegreeVec>;
using Vertex = int;
using Graph = std::map<Vertex, Edges>;

Graph ArrayToGraph(const std::vector<int>& vertexs, const std::vector<std::vector<int> >& edges) {
    Graph graph;
    // init graph
    for (auto& val : vertexs) {
        Edges edges;
        graph[val] = edges; 
    }
    for (const std::vector<int>& vec : edges) {
        // solve OutDegree
        if (graph.find(vec[0]) != graph.end()) {
            InDegreeVec& cur_edges = graph[vec[0]].second;
            cur_edges.push_back(vec[1]);
        }
        // solve InDegree
        if (graph.find(vec[1]) != graph.end()) {
            OutDegreeVec& cur_edges = graph[vec[1]].first;
            cur_edges.push_back(vec[0]);
        }
    }
    return graph;
}

int get_zero_indegree_node(Graph& graph) {
    for (auto& node : graph) {
        if (node.second.second.empty()) {
            graph.erase(node.first);
            return node.first;
        }
    }
    return -1;
}

void delete_indegree_node(Graph& graph, int vertex) {
    for (auto& node : graph) {
        InDegreeVec& indegrees = node.second.second;
        InDegreeVec::iterator it = std::find(indegrees.begin(), indegrees.end(), vertex);
        if (it != indegrees.end()) {
            indegrees.erase(it);
        }
    }
}

std::vector<int> topologic_order_of_kahn(Graph& graph) {
    // get 0 Indegree node
    std::vector<int> order;
    int vertex_num = graph.size();
    while(order.size() < vertex_num) {
        if (graph.empty() && order.size() < vertex_num) {
            order.clear();
            break;
        }
        int cur_vertex = get_zero_indegree_node(graph);

        order.push_back(cur_vertex);

        // delete 
        delete_indegree_node(graph, cur_vertex);
    }
    return order;
}

int main() {
    return 0;
}