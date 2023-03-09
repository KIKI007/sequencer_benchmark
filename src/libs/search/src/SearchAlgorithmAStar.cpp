//
// Created by 汪子琦 on 21.03.22.
//

#include "search/SearchAlgorithmAStar.h"
#include <tbb/parallel_for.h>

namespace search{

double SearchAlgorithmAStar::search(AssemblySequence &sequence) {
    clear();

    createStartPoint();

    while(!queue_.empty())
    {

        HeapNode heapNode = queue_.top();
        PtrS searchNode = searchNodes_[heapNode.searchNodeID];
        queue_.pop();
        searchNode->inQueue = false;

//        std::vector<int> partIDs = stateGraph_->getInstalledParts(stateGraph_->nodes_[searchNode->nodeID]);
//        for(int id = 0; id < partIDs.size(); id++){
//            std::cout << partIDs[id] << " ";
//        }
//        std::cout << std::endl;

        if(termination(searchNode)){
            getSolution(searchNode, sequence);
            return searchNode->currentCost;
        }

        std::vector<PtrS> childNodeIndex;
        expendNode(searchNode, childNodeIndex);

        for(int id = 0; id < childNodeIndex.size(); id++)
        {
            updateNode(searchNode, childNodeIndex[id]);
        }
    }
    return -1.0;
}

void SearchAlgorithmAStar::updateNode(PtrS currNode, PtrS updateNode) {
    double eval = updateNode->eval;
    if(currNode->currentCost + eval < updateNode->currentCost)
    {
        updateNode->currentCost = currNode->currentCost + eval;
        double total_cost = updateNode->currentCost +  updateNode->futureCost;
        if(updateNode->inQueue == false){
            HeapNode heapNode(total_cost);
            heapNode.nParts = updateNode->nParts;
            heapNode.searchNodeID = updateNode->nodeID;
            //updateNode->handle = queue_.push(heapNode);
            updateNode->inQueue = true;
            updateNode->parent = currNode->nodeID;
        }
        else{
//            HeapNode node = *updateNode->handle;
//            node.value = total_cost;
//            node.nParts = updateNode->nParts;
            //queue_.update(updateNode->handle, node);
        }
    }
}


void SearchAlgorithmAStar::expendNode(PtrS searchNode, std::vector<PtrS> &childNodeIndex) {
    int nodeID = searchNode->nodeID;
    std::shared_ptr<StateGraphNode> node = stateGraph_->nodes_[nodeID];
    std::vector<std::shared_ptr<StateGraphNode>> preChildNodes, newChildNodes;
    stateGraph_->expandNode(node, preChildNodes, newChildNodes);

    childNodeIndex.resize(newChildNodes.size());

    tbb::parallel_for( tbb::blocked_range<int>(0,newChildNodes.size()),
                      [&](tbb::blocked_range<int> r)
                      {
                          for (int id =r.begin(); id <r.end(); ++id)
                          {
                              int childNodeID = newChildNodes[id]->nodeID;

                              PtrS newSearchNode = std::make_shared<SearchNode>();
                              newSearchNode->nodeID = childNodeID;
                              newSearchNode->currentCost = 1 << 30;
                              newSearchNode->visited = false;
                              newSearchNode->inQueue = false;
                              newSearchNode->nParts = stateGraph_->getInstalledParts(newChildNodes[id]).size();

                              childNodeIndex[id] = newSearchNode;

                              newSearchNode->futureCost = heuristics(newSearchNode);
                              newSearchNode->eval = evaluation(newSearchNode);
                          }
                      });

    for(int id = 0; id < childNodeIndex.size(); id++){
        searchNodes_.push_back(childNodeIndex[id]);
    }

    for(int id = 0; id < preChildNodes.size(); id++) {
        int childNodeID = preChildNodes[id]->nodeID;
        childNodeIndex.push_back(searchNodes_[childNodeID]);
    }
}

double SearchAlgorithmAStarGreedyHeuristic::heuristics(search::SearchAlgorithm::PtrS searchNode) {
    std::shared_ptr<StateGraph> graph = std::make_shared<StateGraph>(*stateGraph_);
    std::shared_ptr<SearchAlgorithmGreedy> greedy_searcher = std::make_shared<SearchAlgorithmGreedy>(graph);

    std::vector<unsigned > assemblyState = stateGraph_->getAssemblyState(*stateGraph_->nodes_[searchNode->nodeID]);
    graph->addNewNode(assemblyState);
    PtrS newSearchNode = std::make_shared<SearchNode>(*searchNode);
    newSearchNode->nodeID = 0;
    newSearchNode->parent = -1;
    newSearchNode->eval = evaluation(newSearchNode);
    newSearchNode->currentCost = newSearchNode->eval;
    greedy_searcher->searchNodes_.push_back(newSearchNode);
    AssemblySequence sequence;
    double heuristic = greedy_searcher->search(newSearchNode, sequence);
    return heuristic;
}

double SearchAlgorithmAStarBeamSearchHeuristic::heuristics(SearchAlgorithm::PtrS searchNode) {
    std::shared_ptr<StateGraph> graph = std::make_shared<StateGraph>(*stateGraph_);
    std::shared_ptr<SearchAlgorithmBeamSearch> beam_searcher = std::make_shared<SearchAlgorithmBeamSearch>(graph, 10, 0.5);

    std::vector<unsigned > assemblyState = stateGraph_->getAssemblyState(*stateGraph_->nodes_[searchNode->nodeID]);
    graph->addNewNode(assemblyState);
    PtrS newSearchNode = std::make_shared<SearchNode>(*searchNode);
    newSearchNode->nodeID = 0;
    newSearchNode->parent = -1;
    newSearchNode->eval = evaluation(newSearchNode);
    newSearchNode->currentCost = newSearchNode->eval;
    beam_searcher->searchNodes_.push_back(newSearchNode);

    HeapNode heapNode(0.0);
    heapNode.searchNodeID = 0;
    heapNode.nParts = 0;
    beam_searcher->queue_.push(heapNode);

    AssemblySequence sequence;
    double heuristic = beam_searcher->search(newSearchNode, sequence);
    return heuristic;
}

double SearchAlgorithmAStarTopologyOptimization::heuristics(SearchAlgorithm::PtrS searchNode) {
    return 0.0;
}

}