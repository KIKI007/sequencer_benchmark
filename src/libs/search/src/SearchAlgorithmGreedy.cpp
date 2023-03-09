//
// Created by 汪子琦 on 21.03.22.
//

#include "search/SearchAlgorithmGreedy.h"
#include <tbb/parallel_for.h>
namespace search{
double SearchAlgorithmGreedy::search(AssemblySequence &sequence)
{
    timer = tbb::tick_count::now();
    clear();
    if(method == backtrackMethod)
    {
        SearchAlgorithm::createStartPoint();
        double final_cost = std::numeric_limits<double>::max();
        searchDFS(searchNodes_.front(), sequence, final_cost);
        return final_cost;
    }
    else if(method == forwardMethod){
        SearchAlgorithm::createStartPoint();
        return search(searchNodes_.front(), sequence);
    }
    else if(method == backwardMethod){
        return backward(sequence);
    }
    else if(method == randomMethod){
        SearchAlgorithm::createStartPoint();
        return randomSearch(searchNodes_.front(), sequence);
    }
    return std::numeric_limits<double>::max();
}

double SearchAlgorithmGreedy::backward(AssemblySequence &sequence)
{
    createEndNode();
    PtrS searchNode = searchNodes_.front();
    for(int id = 0; id < stateGraph_->nPart_; id++)
    {
        std::vector<PtrS> childNodeIndex;
        expandBackward(searchNode, childNodeIndex);

        double min_value = std::numeric_limits<double>::max();
        PtrS min_searchNode;

        for(int id = 0; id < childNodeIndex.size(); id++)
        {
            double newCost = sum(childNodeIndex[id]->eval, searchNode->currentCost);
            if(min_value > newCost)
            {
                min_value = newCost;
                min_searchNode = childNodeIndex[id];
            }
        }
        min_searchNode->currentCost = sum(min_searchNode->eval, searchNode->currentCost);
        min_searchNode->parent = searchNode->nodeID;
        searchNode = min_searchNode;
    }

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

    std::vector<std::shared_ptr<StateGraphNode>> stateNodes;
    for(int id = 0; id < searchNodes.size(); id++)
    {
        stateNodes.push_back(stateGraph_->nodes_[searchNodes[id]->nodeID]);
    }

    stateGraph_->getSolution(stateNodes, sequence);

    //compute solution
    return searchNode->currentCost;
}

void SearchAlgorithmGreedy::createEndNode()
{
    std::vector<unsigned> assemblyState;
    assemblyState.resize(stateGraph_->nChunk_, 0);

    for(int id = 0; id < stateGraph_->nPart_; id++)
    {
        stateGraph_->setPartMode(id, 1, assemblyState);
    }

    stateGraph_->addNewNode(assemblyState);
    stateGraph_->evaluateNode(stateGraph_->nodes_[0]);
    stateGraph_->nodeEdgeOffsetStart_.push_back(0);
    stateGraph_->nodeEdgeOffsetEnd_.push_back(0);

    HeapNode heapNode(0.0);
    heapNode.searchNodeID = 0;
    heapNode.nParts = stateGraph_->nPart_;
    queue_.push(heapNode);

    PtrS searchNode = std::make_shared<SearchNode>();
    searchNode->nodeID = 0;
    searchNode->inQueue = true;
    searchNode->futureCost = 0.0;
    searchNode->currentCost = stateGraph_->nodes_[0]->cost;
    searchNode->eval = stateGraph_->nodes_[0]->cost;
    searchNode->parent = -1;
    searchNode->nParts = stateGraph_->nPart_;

    searchNodes_.push_back(searchNode);
}

void SearchAlgorithmGreedy::expandBackward(SearchAlgorithm::PtrS searchNode, std::vector<PtrS> &childNodeIndex)
{
    std::vector<int> partIDs = stateGraph_->getInstalledParts(stateGraph_->nodes_[searchNode->nodeID]);

    std::vector<StateGraph::PtrN> childNodes;
    for(int id = 0; id < partIDs.size(); id++)
    {
        std::vector<unsigned> assemblyState;
        assemblyState.resize(stateGraph_->nChunk_, 0);
        for(int jd = 0; jd < partIDs.size(); jd++)
        {
            if(id != jd){
                stateGraph_->setPartMode(partIDs[jd], 1, assemblyState);
            }
        }

        auto [nodeID, newNode] = stateGraph_->addNewNode(assemblyState);
        childNodes.push_back(stateGraph_->nodes_[nodeID]);
    }

    childNodeIndex.clear();
    for(int id = 0; id < childNodes.size(); id++)
    {
        int childNodeID = childNodes[id]->nodeID;

        PtrS newSearchNode = std::make_shared<SearchNode>();
        newSearchNode->nodeID = childNodeID;
        newSearchNode->currentCost = std::numeric_limits<double>::max();
        childNodeIndex.push_back(newSearchNode);
        newSearchNode->eval = evaluation(newSearchNode);
    }

    for(int id = 0; id < childNodeIndex.size(); id++)
    {
        searchNodes_.push_back(childNodeIndex[id]);
    }
}

bool SearchAlgorithmGreedy::searchDFS(SearchAlgorithm::PtrS searchNode, AssemblySequence &sequence, double &final_cost)
{
    double time = (tbb::tick_count::now() - timer).seconds();
    if(time > maxtime) return true;

    if(termination(searchNode) && final_cost > searchNode->currentCost)
    {
        final_cost = searchNode->currentCost;
        sequence.steps.clear();
        getSolution(searchNode, sequence);
//        std::cout << final_cost << std::endl;
        return false;
    }

    std::vector<PtrS> childNodeIndex;
    expendNode(searchNode, childNodeIndex);

    double min_value = std::numeric_limits<double>::max();
    PtrS min_searchNode;

    std::sort(childNodeIndex.begin(), childNodeIndex.end(), [&](PtrS a, PtrS b)->bool {
        return a->eval < b->eval;
    });

    for(int id = 0; id < childNodeIndex.size(); id++)
    {
        PtrS child = childNodeIndex[id];
        child->currentCost = child->eval + searchNode->currentCost;
        child->parent = searchNode->nodeID;
        if(child->currentCost > final_cost) continue ;
        if(searchDFS(child, sequence, final_cost))
            return true;
    }
    return false;
}


double SearchAlgorithmGreedy::search(SearchAlgorithm::PtrS searchNode, AssemblySequence &sequence)
{

    while(!termination(searchNode))
    {
        std::vector<PtrS> childNodeIndex;
        expendNode(searchNode, childNodeIndex);

        double min_value = std::numeric_limits<double>::max();
        PtrS min_searchNode;

        for(int id = 0; id < childNodeIndex.size(); id++)
        {
            double newCost = sum(childNodeIndex[id]->eval, searchNode->currentCost);
            if(min_value > newCost)
            {
                min_value = newCost;
                min_searchNode = childNodeIndex[id];
            }
        }

        min_searchNode->currentCost = sum(min_searchNode->eval, searchNode->currentCost);
        min_searchNode->parent = searchNode->nodeID;
        searchNode = min_searchNode;
    }

    getSolution(searchNode, sequence);

    return searchNode->currentCost;
}


void SearchAlgorithmGreedy::expendNode(PtrS searchNode, std::vector<PtrS> &childNodeIndex) {
    int nodeID = searchNode->nodeID;
    std::shared_ptr<StateGraphNode> node = stateGraph_->nodes_[nodeID];
    std::vector<std::shared_ptr<StateGraphNode>> preChildNodes, newChildNodes;
    stateGraph_->expandNode(node, preChildNodes, newChildNodes);

    childNodeIndex.resize(newChildNodes.size());
    //tbb::parallel_for( tbb::blocked_range<int>(0,newChildNodes.size()),
                      //[&](tbb::blocked_range<int> r)
                      for(int id = 0; id < newChildNodes.size(); id++)
                          //for (int id =r.begin(); id <r.end(); ++id)
                          {
                              int childNodeID = newChildNodes[id]->nodeID;

                              PtrS newSearchNode = std::make_shared<SearchNode>();
                              newSearchNode->nodeID = childNodeID;
                              newSearchNode->currentCost = std::numeric_limits<double>::max();

                              childNodeIndex[id] = newSearchNode;

                              newSearchNode->eval = evaluation(newSearchNode);
                          }
                      //});

    for(int id = 0; id < childNodeIndex.size(); id++) {
        searchNodes_.push_back(childNodeIndex[id]);
    }

    for(int id = 0; id < preChildNodes.size(); id++) {
        int childNodeID = preChildNodes[id]->nodeID;
        searchNodes_[childNodeID]->currentCost = std::numeric_limits<double>::max();
        searchNodes_[childNodeID]->eval = evaluation(searchNodes_[childNodeID]);
        childNodeIndex.push_back(searchNodes_[childNodeID]);
    }
}

double SearchAlgorithmGreedy::randomSearch(SearchAlgorithm::PtrS searchNode, AssemblySequence &sequence) {
    while(!termination(searchNode))
    {
        std::vector<PtrS> childNodeIndex;
        expendNode(searchNode, childNodeIndex);

        double min_value = std::numeric_limits<double>::max();
        PtrS min_searchNode;
        for(auto it = childNodeIndex.begin(); it != childNodeIndex.end();){
            if((*it)->eval > 0.1){
                it = childNodeIndex.erase(it);
            }
            else{
                it++;
            }
        }
        min_searchNode = childNodeIndex[rand() % childNodeIndex.size()];
        min_searchNode->currentCost = sum(min_searchNode->eval, searchNode->currentCost);
        min_searchNode->parent = searchNode->nodeID;
        searchNode = min_searchNode;
    }

    getSolution(searchNode, sequence);

    return searchNode->currentCost;
}
}
