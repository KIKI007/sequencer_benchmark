//
// Created by 汪子琦 on 04.09.22.
//

#ifndef ROBO_CRAFT_STATEGRAPHMULTIPLEARMS_H
#define ROBO_CRAFT_STATEGRAPHMULTIPLEARMS_H
#include "StateGraph.h"
namespace search
{
class StateGraphMultipleArms : public StateGraph
{
public:

    int numArms_;

    std::vector<int> startPartIDs_;

    std::vector<int> endPartIDs_;

    std::vector<int> fixedPartIDs_;

    bool head_ = false;

public:
    StateGraphMultipleArms(std::shared_ptr<PartGraph> partGraph, int numArms)
        : StateGraph(partGraph, 2), numArms_(numArms)
    {
        startPartIDs_ = {};
        fixedPartIDs_ = {};
        for(int id = 0; id < partGraph->nNode(); id++){
            endPartIDs_.push_back(id);
        }
    }

    StateGraphMultipleArms(const StateGraphMultipleArms& graph);

public:

    void evaluateNode(PtrN node) override;

    std::string node_label(PtrN node) override;

    void createRootNode() override;

    bool visibility(int nodeID) override {
        return true;
    }

    bool checkEndNode(PtrN node) override{
        return getInstalledParts(node).size() == endPartIDs_.size();
    }

    void expandNode(StateGraph::PtrN node, std::vector<PtrN> &preChildNodes, std::vector<PtrN> &newChildNodes) override;

    void expandNode(PtrN node, int remainArms, std::map<std::vector<unsigned>, bool, StateCompare> &nodeIncluded, std::vector<PtrN> &preChildNodes, std::vector<PtrN> &newChildNodes);

    void getSolution(std::vector<PtrN> nodes, AssemblySequence &sequence) override;

    std::vector<unsigned> finishedAssemblyState() override;
};
}


#endif  //ROBO_CRAFT_STATEGRAPHMULTIPLEARMS_H
