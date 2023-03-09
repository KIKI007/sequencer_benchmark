//
// Created by 汪子琦 on 21.03.22.
//

#ifndef ROBO_CRAFT_SEARCHALGORITHMGREEDY_H
#define ROBO_CRAFT_SEARCHALGORITHMGREEDY_H

#include "search/SearchAlgorithm.h"
#include "tbb/tick_count.h"
namespace search
{
class SearchAlgorithmGreedy: public SearchAlgorithm{
public:
    enum Method{
        forwardMethod = 0,
        backwardMethod = 1,
        backtrackMethod = 2,
        randomMethod = 3,
    };

    double maxtime = 10;
    tbb::tick_count timer;
    Method method = forwardMethod;

public:

    SearchAlgorithmGreedy(std::shared_ptr<StateGraph> stateGraph)
        : SearchAlgorithm(stateGraph){

    }
public:

    double search(AssemblySequence &sequence) override;

    double search(PtrS searchNode, AssemblySequence &sequence);

    double randomSearch(PtrS searchNode, AssemblySequence &sequence);

    bool searchDFS(PtrS searchNode, AssemblySequence &sequence, double &final_cost);

    double backward(AssemblySequence &sequence);

    void expandBackward(PtrS searchNode, std::vector<PtrS> &childNodeIndex);

    void createEndNode();

    void expendNode(PtrS searchNode, std::vector<PtrS> &childNodeIndex) override;
};
}


#endif  //ROBO_CRAFT_SEARCHALGORITHMGREEDY_H
