//
// Created by 汪子琦 on 21.03.22.
//

#ifndef ROBO_CRAFT_SEARCHALGORITHMASTAR_H
#define ROBO_CRAFT_SEARCHALGORITHMASTAR_H

#include "search/SearchAlgorithm.h"
#include "search/SearchAlgorithmGreedy.h"
#include "search/SearchAlgorithmBeamSearch.h"

namespace search{
class SearchAlgorithmAStar : public SearchAlgorithm{
public:

    SearchAlgorithmAStar(std::shared_ptr<StateGraph> stateGraph): SearchAlgorithm(stateGraph){

    }

public:

    double search(AssemblySequence &sequence) override;

    void updateNode(PtrS currNode, PtrS updateNode);

    virtual double heuristics(PtrS searchNode){return 0.0;}

    void expendNode(PtrS searchNode, std::vector<PtrS> &childNodeIndex) override;
};


class SearchAlgorithmAStarGreedyHeuristic: public SearchAlgorithmAStar{
public:

    SearchAlgorithmAStarGreedyHeuristic(std::shared_ptr<StateGraph> stateGraph): SearchAlgorithmAStar(stateGraph){}

    double heuristics(PtrS searchNode) override;
};


class SearchAlgorithmAStarBeamSearchHeuristic: public SearchAlgorithmAStar{
public:

    SearchAlgorithmAStarBeamSearchHeuristic(std::shared_ptr<StateGraph> stateGraph): SearchAlgorithmAStar(stateGraph){}

    double heuristics(PtrS searchNode) override;
};

class SearchAlgorithmAStarTopologyOptimization: public SearchAlgorithmAStar
{
public:
    SearchAlgorithmAStarTopologyOptimization(std::shared_ptr<StateGraph> stateGraph): SearchAlgorithmAStar(stateGraph){}

    double heuristics(PtrS searchNode) override;
};

}
#endif  //ROBO_CRAFT_SEARCHALGORITHMASTAR_H
