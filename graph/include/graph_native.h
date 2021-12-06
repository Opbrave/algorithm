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
   Edge(std::shared_ptr<Node<Ty>> a, std::shared_ptr<Node<Ty>> b) : node_pair_(std::make_pair(a, b)) {}

   std::shared_ptr<Node<Ty>> GetStartNode() {
     return node_pair_.first;
   }

   std::shared_ptr<Node<Ty>> GetEndNode() {
     return node_pair_.second;
   }

   friend std::ostream& operator<<(std::ostream* os, Edge edge);

  private:
   // A default direction been defined by pair， where pair.first  --> . pair.second
   std::pair<std::shared_ptr<Node<Ty>>, std::shared_ptr<Node<Ty>>> node_pair_;
};


template<typename Ty>
std::ostream& operator<<(std::ostream& os, Edge<Ty> edge) {
  os << "Node(" << edge.GetStartNode()->Name() << ") -----> Node(" << edge.GetEndNode()->Name() << ")"; 
  return os;
}

template<class CNode, class CEdge>
struct Graph {
  public:
   using InEdgeMap = std::map<CNode*, std::set<CNode*> >;
   using OutEdgeMap = InEdgeMap;
   std::shared_ptr<CNode> AddNode(std::string name);

   void AddEdge(CEdge& edge) {
     AddEdge(edge.GetStartNode().get(), edge.GetEndNode().get());
   }


   void AddEdge(std::shared_ptr<CNode>& a, std::shared_ptr<CNode>& b);

   void DumpNodes() {
     for (const auto& item : nodes_)
       LOG(INFO) << "Node: " << item->Name();
   }

   void DumpEdges() {
     for (const auto& edge: edges_) {
       LOG(INFO) << edge;
     }
   }

  private:
   std::vector<std::shared_ptr<CNode> > nodes_;
   std::vector<CEdge> edges_;
   InEdgeMap edges_in_;
   OutEdgeMap edges_out_;

   
};

template<class CNode, class CEdge>
std::shared_ptr<CNode> Graph<CNode, CEdge>::AddNode(std::string name) {
    nodes_.push_back(std::make_shared<CNode>(name));
    return nodes_.back();
}

template<class CNode, class CEdge>
void Graph<CNode, CEdge>::AddEdge(std::shared_ptr<CNode>& start, std::shared_ptr<CNode>& end) {
  // update Node's indegree
  auto& in_set = edges_in_[end.get()];
  in_set.insert(start.get());

  // update Node's outdegree
  auto& out_set = edges_out_[start.get()];
  out_set.insert(end.get());
  edges_.push_back(CEdge(start, end));
}


} // namespace graph
} // namespace algorithm

#endif // __ALGORITHM_NATIVE_GRAPH_CLASS__
