//
// Created by 汪子琦 on 04.09.22.
//

#include "search/StateGraphViz.h"
namespace search
{

void StateGraphViz::createBoostGraph(graph_t &graph)
{
    int num_nodes = nodes_.size();
    std::vector<double> edge_weights;
    std::vector<std::pair<int, int>> edge_array;
    for (int id = 0; id < nodes_.size(); id++) {
        for (int jd = nodeEdgeOffsetStart_[id]; jd < nodeEdgeOffsetEnd_[id]; jd++)
        {
            std::shared_ptr<StateGraphNode> nodeA = nodes_[id];
            std::shared_ptr<StateGraphNode> nodeB = nodes_[edges_[jd]];
            double weight = nodeB->cost;
            edge_array.push_back({nodeB->nodeID, nodeA->nodeID});
            edge_weights.push_back(weight);
        }
    }
    graph = graph_t(edge_array.data(), edge_array.data() + edge_array.size(), edge_weights.data(), num_nodes);
}

std::vector<double> StateGraphViz::computeShortestDistanceSummation()
{
    typedef boost::graph_traits<graph_t>::vertex_descriptor vertex_descriptor;
    graph_t graph;
    createBoostGraph(graph);

    std::vector<vertex_descriptor> p(num_vertices(graph));
    std::vector<double> d(num_vertices(graph));
    vertex_descriptor s = vertex(nodes_.back()->nodeID, graph);
    dijkstra_shortest_paths(graph, s, boost::predecessor_map(&p[0]).distance_map(&d[0]));
    return d;
}

std::vector<double> StateGraphViz::computeShortestDistanceMaximum() {
    struct HeapNode {
        HeapNode(double value, int nodeID) : value(value), nodeID(nodeID) {}
        double value;
        int nodeID;
    };

    struct HeapNodeCompare {
        bool operator()(const HeapNode &n1, const HeapNode &n2) const {
            return n1.value > n2.value;
        }
    };

    typedef boost::heap::fibonacci_heap<HeapNode, boost::heap::compare<HeapNodeCompare>> fibonacci_heap;
    fibonacci_heap  queue;

    graph_t graph;
    createBoostGraph(graph);

    typedef boost::graph_traits<graph_t>::vertex_descriptor vertex_descriptor;
    vertex_descriptor s = vertex(nodes_.back()->nodeID, graph);
    auto weight_map = get(boost::edge_weight, graph);

    std::vector<double> d;
    std::vector<fibonacci_heap ::handle_type> handles;

    for(int id = 0; id < nodes_.size(); id++){
        if(id != s){
            d.push_back(std::numeric_limits<double>::max());
        }
        else{
            d.push_back(0);
        }
        handles.push_back(queue.push(HeapNode(d[id], id)));
    }

    while(!queue.empty()){

        HeapNode node = queue.top();
        queue.pop();
        vertex_descriptor u = node.nodeID;

        auto edges = boost::out_edges(u, graph);
        for(auto edge: boost::make_iterator_range(edges))
        {
            vertex_descriptor v = boost::target(edge, graph);

            double cost = std::max(d[u], weight_map[edge]);
            if(cost < d[v]){
                d[v] = cost;
                queue.update(handles[v], HeapNode(cost, v));
            }
        }
    }
    return d;
}

}
