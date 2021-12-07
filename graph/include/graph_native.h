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
   const std::string Name() const { return name_; }

  private:
   std::string name_;
   Ty Attrs;
};

template<typename Ty>
std::ostream& operator<<(std::ostream& os, const Node<Ty>& node) {
  os << "{ \n" << "\t type: Node\n" << "\t name: " << node.Name() << "\n }\n";
  return os;
}

template<typename Ty>
struct Edge {
  public:
   Edge() = delete;
   Edge(std::shared_ptr<Node<Ty>> a, std::shared_ptr<Node<Ty>> b) : node_pair_(std::make_pair(a, b)) {}

   const std::shared_ptr<Node<Ty>> GetStartNode() const {
     return node_pair_.first;
   }

   const std::shared_ptr<Node<Ty>> GetEndNode() const {
     return node_pair_.second;
   }

  private:
   // A default direction been defined by pair， where pair.first  --> . pair.second
   std::pair<std::shared_ptr<Node<Ty>>, std::shared_ptr<Node<Ty>>> node_pair_;
};


template<typename Ty>
std::ostream& operator<<(std::ostream& os, const Edge<Ty>& edge) {
  os << "Node(" << edge.GetStartNode()->Name() << ") -----> Node(" << edge.GetEndNode()->Name() << ")";
  return os;
}

template<class CNode, class CEdge>
struct Graph {
  public:
   using InEdgeMap = std::map<CNode*, std::set<CNode*> >;
   using OutEdgeMap = InEdgeMap;
   using NodeVec = std::vector<std::shared_ptr<CNode>>;
   std::shared_ptr<CNode> AddNode(std::string name);

   void AddEdge(CEdge& edge) {
     AddEdge(edge.GetStartNode().get(), edge.GetEndNode().get());
   }


   void AddEdge(std::shared_ptr<CNode>& a, std::shared_ptr<CNode>& b);

   CNode* GetNode(size_t index) {
     CHECK_LT(index, nodes_.size());
     return nodes_[index].get();
   }

   std::set<CNode*> GetOutNodes(CNode* node) {
     if (edges_out_.find(node) != edges_out_.end()) {
       return edges_out_[node];
     }
     return {};
   }

   std::set<CNode*> GetInNodes(CNode* node) {
     if (edges_in_.find(node) != edges_in_.end()) {
       return edges_in_[node];
     }
     return {};
   }

   void DumpNodes() {
     for (const auto& item : nodes_)
       LOG(INFO) << "Node: " << item->Name();
   }

   void DumpEdges() {
     for (const auto& edge: edges_) {
       LOG(INFO) << edge;
     }
   }

   NodeVec GetTopoOrder();

  std::set<std::vector<std::shared_ptr<CNode>>> GetSCCs();

  private:

    std::shared_ptr<CNode> get_and_delete_zero_indegree_node(NodeVec& nodes, InEdgeMap& in_edges) {
    for (const auto& node : nodes) {
      LOG(INFO) << "Current node: " << node->Name();
      if (in_edges.find(node.get()) == in_edges.end() || in_edges[node.get()].empty()) {
        typename std::vector<std::shared_ptr<CNode>>::iterator it = std::find(nodes.begin(), nodes.end(), node);
        // TODO: refine it
        // copy node
        auto cpy_node = node;
        // update another node's indegree
        for (auto& item : in_edges) {
          auto& inedge_set = item.second;
          if (inedge_set.find(node.get()) != inedge_set.end()) {
            inedge_set.erase(node.get());
          }
        }
        if (it != nodes.end())
          nodes.erase(it);
        LOG(INFO) << "Entry Node: " << cpy_node->Name();
        return cpy_node;
      } else {
        auto node_set = in_edges[node.get()];
        LOG(INFO) << "Node " << node->Name() << " in edge size: " << node_set.size();
      }
    }
    CHECK(false) << "This Graph has a cycle!";
  };


  private:
   NodeVec nodes_;
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

// kahn algorithm for Topo order
template<class CNode, class CEdge>
std::vector<std::shared_ptr<CNode>> Graph<CNode, CEdge>::GetTopoOrder() {
  // TODO: Should choose a better one
  std::vector<std::shared_ptr<CNode>> mutate_nodes = nodes_;
  InEdgeMap mutate_inedge = edges_in_;
  //OutEdgeMap mutate_outedge = edges_outs_;

  std::vector<std::shared_ptr<CNode>> order;
  int vertex_size = nodes_.size();
  while (order.size() < vertex_size) {
    if (mutate_nodes.empty() && order.size() < vertex_size) {
      order.clear();
      break;
      LOG(INFO) << "Cycle checked!";
    }
    auto entry = get_and_delete_zero_indegree_node(mutate_nodes, mutate_inedge);
    order.push_back(entry);
  }
  return order;

}

// kosaraju algorithm for SCC(Strong Connection Component)
// 1. reverse the graph
// 2. DFS in reverse graph, get a finished time order
// 3. DFS with a decreasing order calculated in step2
//template<class CNode, class CEdge>
//std::set<std::vector<std::shared_ptr<CNode>>> Graph<CNode, CEdge>::GetSCCs() {
//
//}



// Algorithms outside graph struct

template <typename Ty>
void DFS(Graph<Node<Ty>, Edge<Ty>> &graph, Node<Ty> *node, std::vector<Node<Ty> *> &node_vec)
{
  auto outs = graph.GetOutNodes(node);
  for (auto &vertex : outs)
  {
    DFS(graph, vertex, node_vec);
  }
  if (std::find(node_vec.begin(), node_vec.end(), node) == node_vec.end())
  {
    node_vec.push_back(node);
  }
}

template <typename Ty>
void BFS(Graph<Node<Ty>, Edge<Ty>> &graph, Node<Ty> *node, std::vector<Node<Ty> *> &node_vec)
{
}

} // namespace graph
} // namespace algorithm

#endif // __ALGORITHM_NATIVE_GRAPH_CLASS__
