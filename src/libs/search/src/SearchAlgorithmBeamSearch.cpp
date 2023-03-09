//
// Created by 汪子琦 on 23.03.22.
//

#include "search/SearchAlgorithmBeamSearch.h"
#include <tbb/parallel_for.h>
#include "tbb/tick_count.h"
#include "search/StateGraphDynamicScaffold.h"
namespace search
{

double SearchAlgorithmBeamSearch::search(SearchAlgorithm::PtrS searchNode, AssemblySequence& sequence) {
    int step = 0;

    PtrS final_node;
    double final_cost = std::numeric_limits<double>::max();
    candidates_.push_back(queue_.top());
    while(!candidates_.empty())
    {
        std::sort(candidates_.begin(), candidates_.end(), [&](auto A, auto B)-> bool{
                return A.value < B.value;
            });
        std::vector<HeapNode> topNodes;

//       std::cout << "step" << step << std::endl;
//        for(int id = 0; id < candidates_.size(); id++)
//        {
//            auto u = candidates_[id];
//            auto startPartIDs = stateGraph_->getInstalledParts(stateGraph_->nodes_[0]);
//            auto partIDs = stateGraph_->getInstalledParts(stateGraph_->nodes_[u.searchNodeID]);
//
//            auto fixedPartIDs = std::static_pointer_cast<StateGraphDynamicScaffold>(stateGraph_)->getFixedParts(stateGraph_->nodes_[u.searchNodeID]);
//
//            for(int jd = 0; jd < partIDs.size(); jd++)
//            {
//                //if(find(startPartIDs.begin(), startPartIDs.end(), partIDs[jd]) == startPartIDs.end())
//                {
//                    std::cout << partIDs[jd];
//                    if(find(fixedPartIDs.begin(), fixedPartIDs.end(), partIDs[jd]) != fixedPartIDs.end()){
//                        std::cout << "* ";
//                    }
//                    else{
//                        std::cout << " ";
//                    }
//                }
//            }
//            std::cout << ":" << u.value << ", " << searchNodes_[u.searchNodeID]->eval << std::endl;
//        }
//        std::cout << std::endl;

        for(int id = 0; id < maxLayerNodeExplore && id < candidates_.size(); id++)
        {
            auto u = candidates_[id];
            std::vector<int> partIDs = stateGraph_->getInstalledParts(stateGraph_->nodes_[u.searchNodeID]);
            topNodes.push_back(u);
        }
        step++;

        candidates_.clear();
        searchNodeInQueue_.clear();

        for(int id = 0; id < topNodes.size(); id++)
        {
            PtrS searchNode = searchNodes_.at(topNodes[id].searchNodeID);
            std::vector<PtrS> childNodeIndex;
            expendNode(searchNode, childNodeIndex);

            for(int jd = 0; jd < childNodeIndex.size(); jd++)
            {
                if(childNodeIndex[jd]->eval < deformation_prune_
                    || childNodeIndex[jd]->nParts == stateGraph_->nPart_)
                {
                    updateNode(searchNode, childNodeIndex[jd]);
                    if(termination(childNodeIndex[jd]))
                    {
                        if(final_cost > childNodeIndex[jd]->currentCost)
                        {
                            final_cost = childNodeIndex[jd]->currentCost;
                            final_node = childNodeIndex[jd];
                        }
                    }
                }
            }
        }
    }

    if(final_node)
    {
        getSolution(final_node, sequence);
        return final_cost;
    }

    return std::numeric_limits<double>::max();
}

double SearchAlgorithmBeamSearch::search(AssemblySequence& sequence) {
    clear();
    createStartPoint();
    return search(searchNodes_.front(), sequence);
}


void SearchAlgorithmBeamSearch::updateNode(SearchAlgorithm::PtrS currNode, SearchAlgorithm::PtrS updateNode)
{
    double edgeCost = stateGraph_->evaluateNode2Node(stateGraph_->nodes_[currNode->nodeID],
                                                     stateGraph_->nodes_[updateNode->nodeID]);

    double newCost = sum(edgeCost, sum(currNode->currentCost, updateNode->eval));

    if(updateNode->currentCost > newCost)
    {
        updateNode->currentCost = newCost;
        updateNode->parent = currNode->nodeID;

        if(searchNodeInQueue_.find(updateNode->nodeID) != searchNodeInQueue_.end())
        {
            int queueID = searchNodeInQueue_[updateNode->nodeID];
            candidates_[queueID].value = updateNode->currentCost;
        }
    }


    if(searchNodeInQueue_.find(updateNode->nodeID) == searchNodeInQueue_.end())
    {
        HeapNode heapNode(updateNode->currentCost);
        heapNode.searchNodeID = updateNode->nodeID;
        searchNodeInQueue_[updateNode->nodeID] = candidates_.size();
        candidates_.push_back(heapNode);
    }
}

void SearchAlgorithmBeamSearch::expendNode(PtrS searchNode, std::vector<PtrS> &childNodeIndex) {

    int nodeID = searchNode->nodeID;
    std::shared_ptr<StateGraphNode> node = stateGraph_->nodes_[nodeID];
    std::vector<std::shared_ptr<StateGraphNode>> preChildNodes, newChildNodes;

    auto timer = tbb::tick_count::now();

    stateGraph_->expandNode(node, preChildNodes, newChildNodes);

    timer = tbb::tick_count::now();
    childNodeIndex.resize(newChildNodes.size());

    tbb::parallel_for( tbb::blocked_range<int>(0,newChildNodes.size()),
                      [&](tbb::blocked_range<int> r)
                      {
                          for (int id =r.begin(); id <r.end(); ++id)
//                          for(int id = 0; id < newChildNodes.size(); id++)
                          {
                              int childNodeID = newChildNodes[id]->nodeID;

                              PtrS newSearchNode = std::make_shared<SearchNode>();
                              newSearchNode->nodeID = childNodeID;
                              newSearchNode->nParts = stateGraph_->getInstalledParts(newChildNodes[id]).size();
                              newSearchNode->currentCost = std::numeric_limits<double>::max();

                              childNodeIndex[id] = newSearchNode;

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

}

