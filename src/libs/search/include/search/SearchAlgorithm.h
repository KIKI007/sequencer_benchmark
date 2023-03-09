//
// Created by 汪子琦 on 16.03.22.
//

#ifndef ROBO_CRAFT_SEARCHALGORITHM_H
#define ROBO_CRAFT_SEARCHALGORITHM_H

//#include <boost/heap/fibonacci_heap.hpp>
#include <map>
#include <queue>
#include <set>

#include "AssemblySequence.h"
#include "StateGraph.h"
#include "frame/FrameAssembly.h"

namespace search {
struct HeapNode {
    HeapNode(double value) : value(value) {}
    double value;
    int nParts;
    int searchNodeID;
};

struct HeapNodeCompare
{
    bool operator()(const HeapNode &n1, const HeapNode &n2) const
    {
        return n1.value > n2.value;
    }
};

//typedef boost::heap::fibonacci_heap<HeapNode, boost::heap::compare<HeapNodeCompare>> fibonacci_heap;

typedef std::priority_queue<HeapNode, std::vector<HeapNode>, HeapNodeCompare> fibonacci_heap;

struct SearchNode
{
    int parent;
    double currentCost;
    double futureCost;
    double eval;
    bool inQueue;
    bool visited;
    int nodeID;
    int nParts;
    //fibonacci_heap::handle_type handle;
};

class SearchAlgorithm
{
public:

    typedef std::shared_ptr<SearchNode> PtrS;

    std::vector<std::shared_ptr<SearchNode>> searchNodes_;

    std::shared_ptr<StateGraph> stateGraph_;

    fibonacci_heap queue_;

    double use_max_operator_ = true;

public:
    SearchAlgorithm(std::shared_ptr<StateGraph> stateGraph) : stateGraph_(stateGraph) {}

public:

    void clear() {
        queue_ = fibonacci_heap();
        searchNodes_.clear();
        stateGraph_->clear();
    }

    virtual double search(AssemblySequence &sequence){return -1.0;}

    void getSolution(PtrS searchNode, AssemblySequence &sequence);

    virtual double evaluation(PtrS searchNode);

    virtual void createStartPoint();

    virtual bool termination(PtrS searchNode);

    virtual void expendNode(PtrS searchNode, std::vector<PtrS> &childNodeIndex){};

    double sum(double a, double b){
        if(use_max_operator_){
            return std::max(a, b);
        }
        else{
            return  a + b;
        }
    }
};
}

#endif  //ROBO_CRAFT_SEARCHALGORITHM_H
