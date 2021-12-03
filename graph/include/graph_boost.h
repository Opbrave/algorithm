/*!
 * @brief Base graph class with Boost
 */

#ifndef __ALGORITHM_GRAPH_CLASS_BOOST_H__
#define __ALGORITHM_GRAPH_CLASS_BOOST_H__

#include <string>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>

namespace algorithm {
namespace graph {
namespace bt {

struct Node {
  uint32_t uid;
  std::string name;
};

struct Edge {
  std::string name;
};

typedef boost::adjacency_list<boost::listS, boost::vecS, boost::directedS, Node,
                              Edge>
    Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor node_t;
typedef boost::graph_traits<Graph>::edge_descriptor edge_t;
typedef boost::graph_traits<Graph>::vertex_iterator vertex_iter;

typedef std::list<node_t> MakeOrder;
// A base class to define APIs
class GraphDerived {
 public:
  GraphDerived(){};

  node_t AddNode(const Node& node);
  void AddEdge(node_t u, node_t v);

  uint32_t NodeNum() {
    std::pair<vertex_iter, vertex_iter> vp;
    vp = boost::vertices(graph_);
    return std::distance(vp.first, vp.second);
  }

  Node GetNode(node_t des) {
      return graph_[des];
  }


  MakeOrder TopoOrder();

  void DumpGraph();

 private:
  Graph graph_;
};

uint32_t UniqueID();

Node CreateNode(std::string name);

std::ostream& operator<<(std::ostream& os, Node node);

}  // namespace bt
}  // namespace graph
}  // namespace algorithm

#endif  // __ALGORITHM_GRAPH_CLASS_BOOST_H__
