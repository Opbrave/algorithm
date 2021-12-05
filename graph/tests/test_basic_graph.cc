#include "graph_native.h"

using namespace algorithm::graph;

int main() {

    Graph<Node<int>, Edge<int>> graph;
    auto node_tmp = std::make_shared<Node<int>>("B");
    auto node_a = graph.AddNode("A");
    graph.DumpNodes();
    return 0;
}

