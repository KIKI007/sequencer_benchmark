//
// Created by 汪子琦 on 16.03.22.
//

#include "search/SearchAlgorithm.h"

namespace search {

double SearchAlgorithm::evaluation(PtrS searchNode) {
    int nodeID = searchNode->nodeID;
    std::shared_ptr<StateGraphNode> stateNode = stateGraph_->nodes_[nodeID];
    if(use_max_operator_ && stateGraph_->checkEndNode(stateNode)){
        return 0;
    }
    else{
        stateGraph_->evaluateNode(stateNode);
        return stateNode->cost;
    }
}

void SearchAlgorithm::createStartPoint() {
    stateGraph_->createRootNode();

    HeapNode heapNode(0.0);
    heapNode.searchNodeID = 0;
    heapNode.nParts = 0;
    queue_.push(heapNode);

    PtrS searchNode = std::make_shared<SearchNode>();
    searchNode->nodeID = 0;
    searchNode->inQueue = true;
    searchNode->futureCost = 0.0;
    searchNode->currentCost = stateGraph_->nodes_[0]->cost;
    searchNode->eval = stateGraph_->nodes_[0]->cost;
    searchNode->parent = -1;
    searchNode->nParts = 0;

    searchNodes_.push_back(searchNode);
}

bool SearchAlgorithm::termination(PtrS searchNode)
{
    int nodeID = searchNode->nodeID;
    std::shared_ptr<StateGraphNode> stateNode = stateGraph_->nodes_[nodeID];
    return stateGraph_->checkEndNode(stateNode);
}

void SearchAlgorithm::getSolution(SearchAlgorithm::PtrS searchNode, AssemblySequence &sequence)
{
    double cost = 0.0;
    PtrS node = searchNode;
    std::vector<std::shared_ptr<SearchNode>> searchNodes;
    while (true)
    {
        std::vector<int> nodePartIDs = stateGraph_->getInstalledParts(stateGraph_->nodes_[node->nodeID]);
        searchNodes.push_back(node);
        if(node->parent == -1)
        {
            break;
        }
        else{
            PtrS parentNode = searchNodes_[node->parent];
            node = parentNode;
        }
    }
    std::reverse(searchNodes.begin(), searchNodes.end());

    std::vector<std::shared_ptr<StateGraphNode>> stateNodes;
    for(int id = 0; id < searchNodes.size(); id++)
    {
        stateNodes.push_back(stateGraph_->nodes_[searchNodes[id]->nodeID]);
    }

    stateGraph_->getSolution(stateNodes, sequence);
    return;
}
}




