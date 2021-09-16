#include "graph.h"

#include<iostream>

#define OUT(str) \
std::cout << str << std::endl

int main() {
    algorithm::graph::GraphDerived graph;
    auto first_vex = graph.AddNode(algorithm::graph::CreateNode("first"));
    auto second_vex = graph.AddNode(algorithm::graph::CreateNode("seconde"));
    auto third_vex = graph.AddNode(algorithm::graph::CreateNode("third"));

    graph.AddEdge(first_vex, second_vex);
    graph.AddEdge(second_vex, third_vex);

    std::cout << "Graph vertex number: " << graph.NodeNum() << std::endl;

    graph.DumpGraph();


    // get topological order
    algorithm::graph::MakeOrder sort_order = graph.TopoOrder();

    std::cout << "====Topological Order====" << std::endl;
    for (auto it = sort_order.begin(); it != sort_order.end(); it++) {
        OUT(graph.GetNode(*it));
    }

    return 0;
}