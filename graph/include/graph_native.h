/*!
 * @brief Native Graph Base
 */

#ifndef __ALGORITHM_NATIVE_GRAPH_CLASS__
#define __ALGORITHM_NATIVE_GRAPH_CLASS__

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <iostream>
#include <map>
#include <set>

#include <glog/logging.h>

namespace algorithm {
namespace graph {

template<typename Ty>
struct Node {
  public:
   Node(std::string name):name_(name) {}
   std::string Name() { return name_; }

  private:
   std::string name_;
   Ty Attrs;
};

template<typename Ty>
struct Edge {
  public:
   Edge() = delete;
   Edge(std::shared_ptr<Node<Ty>> a, std::shared_ptr<Node<Ty>> b);

   std::shared_ptr<Node<Ty>> GetStartNode() {
     return node_pair_.first;
   }

   std::shared_ptr<Node<Ty>> GetEndNode() {
     return node_pair_.second;
   }


  private:
   // A default direction been defined by pair， where pair.first  --> . pair.second
   std::pair<std::shared_ptr<Node<Ty>>, std::shared_ptr<Node<Ty>>> node_pair_;
};

template<class CNode, class CEdge>
struct Graph {
  public:
   using InEdgeMap = std::map<CNode*, std::set<CNode*> >;
   using OutEdgeMap = InEdgeMap;
   std::shared_ptr<CNode> AddNode(std::string name);

   void AddEdge(CEdge& edge);

   void AddEdge(CNode* a, CNode* b);

   void DumpNodes() {
     for (const auto& item : nodes_)
       LOG(INFO) << "Node: " << item->Name();
   }

  private:
   std::vector<std::shared_ptr<CNode> > nodes_;
   InEdgeMap edges_in_;
   OutEdgeMap edges_out_;

   
};

template<class CNode, class CEdge>
std::shared_ptr<CNode> Graph<CNode, CEdge>::AddNode(std::string name) {
    nodes_.push_back(std::make_shared<CNode>(name));
    return nodes_.back();
}

template<class CNode, class CEdge>
void Graph<CNode, CEdge>::AddEdge(CEdge& edge) {
  // update Node's indegree
  auto& in_set = edges_in_[edge.GetEndNode().get()];
  in_set.insert(edge.GetStartNode().get());

  // update Node's outdegree

}


} // namespace graph
} // namespace algorithm

#endif // __ALGORITHM_NATIVE_GRAPH_CLASS__
