#include "graph.h"

#include <iostream>

namespace algorithm {
namespace graph {

node_t GraphDerived::AddNode(const Node& node) {
  node_t graph_node = boost::add_vertex(graph_);
  graph_[graph_node] = node;
  return graph_node;
}

void GraphDerived::AddEdge(node_t u, node_t v) {
  boost::add_edge(u, v, graph_);
}

MakeOrder GraphDerived::TopoOrder() {
    MakeOrder make_order;
    boost::topological_sort(graph_, std::front_inserter(make_order));
    return make_order;
}

void GraphDerived::DumpGraph() {
    std::pair<vertex_iter, vertex_iter> vp;
    vp = boost::vertices(graph_);
    for (; vp.first != vp.second; vp.first++) {
        std::cout << graph_[*vp.first] << std::endl;
    }
}

uint32_t UniqueID() {
  static uint32_t id = 0;
  return id++;
}

Node CreateNode(std::string name) {
  auto uid = UniqueID();
  return Node{uid, name};
}

std::ostream& operator<<(std::ostream& os, Node node) {
    os << "Node id: " << node.uid << " with name: " << node.name;
    return os;
}



}  // namespace graph
}  // namespace algorithm