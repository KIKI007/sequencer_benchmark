//
// Created by 汪子琦 on 04.09.22.
//

#ifndef ROBO_CRAFT_PARTGRAPH_H
#define ROBO_CRAFT_PARTGRAPH_H
#include <vector>
#include <set>
#include <string>
#include <fstream>
#include <Eigen/Dense>
#include <memory>
namespace search
{

class PartGraphNode{
public:
    unsigned partID;
    bool grounded;
    bool visited;
    Eigen::Vector3d color;
};

class PartGraphEdge{
public:
    std::vector<unsigned> partIDs;
};

class PartGraph
{
protected:

    std::vector<std::shared_ptr<PartGraphNode>> nodes_;

    std::vector<std::shared_ptr<PartGraphEdge>> edges_;

public:

    std::vector<int> static_scaffold_partIDs_;

public:

    PartGraph(){}

    PartGraph(const PartGraph &graph);

public:

    int nNode() const {return nodes_.size();}

    void clearVisit();

    void registerVisit(const std::vector<int> &nodeIDs);

    void computeNodesNeighbour(const std::vector<int>& process_nodes, std::vector<int> &neighbourNodeIDs);

    void computeNodesNeighbour(const std::vector<int>& installed_nodes, const std::vector<int>& nodes_in_consider, std::vector<int> &neighbourNodeIDs);

    void computeNodesNeighbour(const std::vector<int>& process_nodes, std::vector<int> &neighbourNodeIDs, int nRing);

    void writeDotGraph(std::string filename);

    std::string DotGraphString();

    std::vector<std::vector<int>> getEdgeData();

    std::vector<unsigned > computeNodeNeighbour(int partID){return edges_[partID]->partIDs;}

public:

    virtual double evaluateStability(const std::vector<int> &subPartIDs, const std::vector<int> &fixedPartIDs){
        return 0.0;
    }

};



}

#endif  //ROBO_CRAFT_PARTGRAPH_H
