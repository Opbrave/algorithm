#include "graph_native.h"

namespace algorithm {
namespace graph {

template<typename Ty>
std::shared_ptr<Node<Ty>> Graph<Ty>::AddNode(std::string name) {
    nodes_.push_back(std::make_shared<Node<Ty>>(name));
    return nodes_.back();
}

template<typename Ty>
void Graph<Ty>::AddEdge(Edge<Ty>& edge) {
    edges_.push_back(edge);
}

template<typename Ty>
void Graph<Ty>::AddEdge(Node<Ty>* a, Node<Ty>* b) {
    edges_.empalce_back(a, b);
}

} // namespace graph
} // namespace algorithm