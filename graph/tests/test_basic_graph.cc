#include "graph_native.h"

using namespace algorithm::graph;


/* Create kinds of graph
 *      A
 *     /  \
 *    B    C
 *     \  /
 *       D
 */
template<typename Ty>
Graph<Node<Ty>, Edge<Ty>> create_pattern1_graph() {
    Graph<Node<int>, Edge<int>> graph;
    auto node_a = graph.AddNode("A");
    auto node_b = graph.AddNode("B");
    auto node_c = graph.AddNode("C");
    auto node_d = graph.AddNode("D");
    graph.AddEdge(node_a, node_b);
    graph.AddEdge(node_a, node_c);
    graph.AddEdge(node_b, node_d);
    graph.AddEdge(node_c, node_d);
    return graph;
}

int main() {

    auto graph_pattern0 = create_pattern1_graph<int>();
    graph_pattern0.DumpNodes();
    graph_pattern0.DumpEdges();
    auto topo_vec = graph_pattern0.GetTopoOrder();
    return 0;
}

