//
// Created by 汪子琦 on 04.09.22.
//

#include "search/StateGraphMultipleArms.h"

namespace search
{

void StateGraphMultipleArms::evaluateNode(PtrN node)
{
    std::vector<int> subset_beams_index = getInstalledParts(node);
    node->cost = partGraph_->evaluateStability(subset_beams_index, fixedPartIDs_);
}

std::string StateGraphMultipleArms::node_label(PtrN node)
{
    std::vector<int> parts = getInstalledParts(node);
    std::string node_label = "\"[";
    for (int id = 0; id < parts.size(); id++) {
        node_label += std::to_string(parts[id]);
        if (id + 1 != parts.size()) {
            node_label += ", ";
        }
    }
    node_label += "]";
    double deformation = node->cost;
    char deformation_str[256];
    sprintf(deformation_str, "%.4lf", deformation);
    std::string deformation_string(deformation_str);
    node_label += "\n" + deformation_string;
    node_label += "\"";
    return node_label;
}

void StateGraphMultipleArms::getSolution(std::vector<PtrN> nodes, AssemblySequence &sequence)
{
    sequence.steps.clear();
    //nothing
    //AssemblyStep step;
    //sequence.steps.push_back(step);

    if(head_){
        AssemblyStep step;
        std::vector<int> currNodePartIDs = getInstalledParts(nodes[0]);
        step.installPartIDs = currNodePartIDs;
        step.holdPartIDs = fixedPartIDs_;
        sequence.steps.push_back(step);
    }

    for (int id = 1; id < nodes.size(); id++)
    {
        PtrN prevNode = nodes[id - 1];
        PtrN currNode = nodes[id];

        std::vector<int> prevNodePartIDs = getInstalledParts(prevNode);
        std::vector<int> currNodePartIDs = getInstalledParts(currNode);
        std::vector<int> newInstalledParts;
        std::sort(currNodePartIDs.begin(), currNodePartIDs.end());
        std::sort(prevNodePartIDs.begin(), prevNodePartIDs.end());
        std::set_difference(currNodePartIDs.begin(), currNodePartIDs.end(), prevNodePartIDs.begin(), prevNodePartIDs.end(),
                            std::back_inserter(newInstalledParts));

//        for (int jd = 0; jd < currNodePartIDs.size(); jd++) {
//            std::cout << currNodePartIDs[jd] << " ";
//        }
//        std::cout << ":\t" << currNode->cost << std::endl;

        AssemblyStep step;
        for (int jd = 0; jd < newInstalledParts.size(); jd++) {
            step.installPartIDs.push_back(newInstalledParts[jd]);
        }
        step.holdPartIDs = fixedPartIDs_;
        sequence.steps.push_back(step);
    }
}

StateGraphMultipleArms::StateGraphMultipleArms(const StateGraphMultipleArms &stateGraph) {
    partGraph_ = stateGraph.partGraph_;
    numArms_ = stateGraph.numArms_;
    nPart_ = stateGraph.nPart_;
    nChunk_ = stateGraph.nChunk_;
    nMode_ = stateGraph.nMode_;
    startPartIDs_ = stateGraph.startPartIDs_;
    endPartIDs_ = stateGraph.endPartIDs_;
    clear();
}

void StateGraphMultipleArms::expandNode(StateGraph::PtrN node, std::vector<PtrN> &preChildNodes, std::vector<PtrN> &newChildNodes) {
    std::map<std::vector<unsigned>, bool, StateCompare> nodeIncluded;
    StateGraphMultipleArms::expandNode(node, numArms_, nodeIncluded, preChildNodes, newChildNodes);
    return;
}

void StateGraphMultipleArms::expandNode(PtrN node,
                                        int remainArms,
                                        std::map<std::vector<unsigned>, bool, StateCompare> &nodeIncluded,
                                        std::vector<PtrN> &preChildNodes,
                                        std::vector<PtrN> &newChildNodes)
{
    if (remainArms == 0)
        return;

    std::vector<int> visited_nodeIDs = getInstalledParts(node);

    std::vector<int> neighbourIDs;
    partGraph_->computeNodesNeighbour(visited_nodeIDs, endPartIDs_,  neighbourIDs);

    for (int id = 0; id < neighbourIDs.size(); id++)
    {
        int partID = neighbourIDs[id];
        std::vector<unsigned> newAssemblyState = setPartMode(partID, 1, *node);

        if (nodeIncluded.find(newAssemblyState) != nodeIncluded.end()) continue;
        nodeIncluded[newAssemblyState] = true;
        auto [newNode, createNewNode] = addNewNode(newAssemblyState);

        if (createNewNode) {
            newChildNodes.push_back(nodes_[newNode]);
        } else {
            preChildNodes.push_back(nodes_[newNode]);
        }

        StateGraphMultipleArms::expandNode(nodes_[newNode], remainArms - 1, nodeIncluded, preChildNodes, newChildNodes);
    }
}

void StateGraphMultipleArms::createRootNode()
{
    std::vector<unsigned> assemblyState;
    assemblyState.resize(nChunk_, 0);

    for(int id = 0; id < startPartIDs_.size(); id++){
        setPartMode(startPartIDs_[id], 1, assemblyState);
    }

    for(int id = 0; id < fixedPartIDs_.size(); id++)
    {
        setPartMode(fixedPartIDs_[id], 1, assemblyState);
    }

    addNewNode(assemblyState);

    evaluateNode(nodes_[0]);

    nodeEdgeOffsetStart_.push_back(0);

    nodeEdgeOffsetEnd_.push_back(0);
}

std::vector<unsigned> StateGraphMultipleArms::finishedAssemblyState() {
    std::vector<unsigned> fullAssemblyState;
    fullAssemblyState.resize(nChunk_, 0);
    for(int id = 0 ;id < endPartIDs_.size(); id++){
        setPartMode(endPartIDs_[id], 1, fullAssemblyState);
    }
    return fullAssemblyState;
}

}

