/*!
 * @brief Native Graph Base
 */

#ifndef __ALGORITHM_NATIVE_GRAPH_CLASS__
#define __ALGORITHM_NATIVE_GRAPH_CLASS__

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace algorithm {
namespace graph {

template<typename Ty>
struct Node {
  public:
   Node(std::string name):name_(name) {}

  private:
   std::string name_;
   Ty Attrs;
};

template<typename Ty>
struct Edge {
  public:
   Edge() = delete;
   Edge(Node<Ty>* a, Node<Ty>* b);

  private:
   std::pair<Node<Ty>*, Node<Ty>*> node_pair_;
};

template<typename Ty>
struct Graph {
  public:
   std::shared_ptr<Node<Ty>> AddNode(std::string name);

   void AddEdge(Edge<Ty>& edge);

   void AddEdge(Node<Ty>* a, Node<Ty>* b);

  private:
   std::vector<std::shared_ptr<Node<Ty>>> nodes_;
   std::vector<Edge<Ty>> edges_;
};

} // namespace graph
} // namespace algorithm

#endif // __ALGORITHM_NATIVE_GRAPH_CLASS__
