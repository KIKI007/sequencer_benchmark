//
// Created by 汪子琦 on 04.09.22.
//

#include "search/PartGraph.h"
namespace search 
{

PartGraph::PartGraph(const PartGraph& graph)
{
    for(int id = 0; id < graph.edges_.size(); id++){
        edges_.push_back(std::make_shared<PartGraphEdge>(*graph.edges_[id]));
    }
    for(int id = 0; id < graph.nodes_.size(); id++){
        nodes_.push_back(std::make_shared<PartGraphNode>(*graph.nodes_[id]));
    }
}

void PartGraph::clearVisit() {
    for (int id = 0; id < nodes_.size(); id++) {
        nodes_[id]->visited = false;
    }
}

void PartGraph::registerVisit(const std::vector<int>& nodeIDs) {
    for (int id = 0; id < nodeIDs.size(); id++) {
        int nID = nodeIDs[id];
        nodes_[nID]->visited = true;
    }
}

void PartGraph::computeNodesNeighbour(const std::vector<int>& process_nodes, std::vector<int>& neighbourNodeIDs, int nRing) {
    clearVisit();
    registerVisit(process_nodes);

    std::vector<int> searchNodes = process_nodes;
    for (int iR = 0; iR < nRing; iR++) {
        std::vector<int> resultNodes;
        computeNodesNeighbour(searchNodes, resultNodes);
        registerVisit(resultNodes);
        neighbourNodeIDs.insert(neighbourNodeIDs.end(), resultNodes.begin(), resultNodes.end());
        searchNodes = resultNodes;
    }

    for (int id = 0; id < nodes_.size(); id++) {
        if (!nodes_[id]->visited && nodes_[id]->grounded) {
            neighbourNodeIDs.push_back(id);
        }
    }
}

void PartGraph::computeNodesNeighbour(const std::vector<int>& process_nodes, std::vector<int>& neighbourNodeIDs) {
    std::set<int> neighbour_set;

    for (int id = 0; id < process_nodes.size(); id++) {
        int currNodeID = process_nodes[id];
        for (int jd = 0; jd < edges_[currNodeID]->partIDs.size(); jd++) {
            int neighbourNodeID = edges_[currNodeID]->partIDs[jd];
            if (nodes_[neighbourNodeID]->visited == false) {
                neighbour_set.insert(neighbourNodeID);
            }
        }
    }

    for (auto it = neighbour_set.begin(); it != neighbour_set.end(); it++) {
        neighbourNodeIDs.push_back(*it);
    }

    return;
}

void PartGraph::computeNodesNeighbour(const std::vector<int>& installed_nodes,
                                      const std::vector<int>& nodes_in_consider,
                                      std::vector<int>& neighbourNodeIDs) {
    std::vector<int> candidate_nodes;
    computeNodesNeighbour(installed_nodes, candidate_nodes, 1);

    neighbourNodeIDs.clear();
    for (int id = 0; id < candidate_nodes.size(); id++)
    {
        auto find_it = std::find(nodes_in_consider.begin(), nodes_in_consider.end(), candidate_nodes[id]);
        if (find_it != nodes_in_consider.end()) {
            neighbourNodeIDs.push_back(candidate_nodes[id]);
        }
    }
    return;
}

void PartGraph::writeDotGraph(std::string filename) {
    std::ofstream fout(filename);
    std::string buffer = DotGraphString();
    fout << buffer;
    fout.close();
}
std::string PartGraph::DotGraphString() {
    //assign color
    for (int id = 0; id < nodes_.size(); id++) {
        if (nodes_[id]->grounded) {
            nodes_[id]->color = Eigen::Vector3d(0.5, 0.5, 0.5);
        } else if (nodes_[id]->visited) {
            nodes_[id]->color = Eigen::Vector3d(1.0, 0.0, 1.0);
        } else {
            nodes_[id]->color = Eigen::Vector3d(1, 1, 1);
        }
    }

    std::stringstream buffer;

    buffer << "graph {\n";
    for (int id = 0; id < nodes_.size(); id++) {
        //fout << id << "[label = \" " << id << ": (" << nodes_[id].end_u << ", " << nodes_[id].end_v << ") \" ";
        buffer << id << "[label = \" " << id << "\" ";
        int r = 255 * nodes_[id]->color[0];
        int g = 255 * nodes_[id]->color[1];
        int b = 255 * nodes_[id]->color[2];

        char color_string[256];
        sprintf(color_string, "\"#%02X%02X%02X \"", r, g, b);

        buffer << ", style=filled,  fillcolor = ";
        buffer << color_string;
        buffer << "]\n";
    }

    for (int id = 0; id < edges_.size(); id++) {
        for (int jd = 0; jd < edges_[id]->partIDs.size(); jd++) {
            if (id < edges_[id]->partIDs[jd])
                buffer << id << "--" << edges_[id]->partIDs[jd] << std::endl;
        }
    }
    buffer << "}\n";
    return buffer.str();
}
std::vector<std::vector<int>> PartGraph::getEdgeData() {
    std::vector<std::vector<int>> edgeData;
    edgeData.resize(2);
    for (int id = 0; id < edges_.size(); id++) {
        for (int jd = 0; jd < edges_[id]->partIDs.size(); jd++) {
            if (id < edges_[id]->partIDs[jd]) {
                edgeData[0].push_back(id);
                edgeData[1].push_back(edges_[id]->partIDs[jd]);
            }
        }
    }

    return edgeData;
}
}