//
// Created by 汪子琦 on 23.03.22.
//

#ifndef ROBO_CRAFT_SEARCHALGORITHMBEAMSEARCH_H
#define ROBO_CRAFT_SEARCHALGORITHMBEAMSEARCH_H
#include "SearchAlgorithm.h"
namespace search{
class SearchAlgorithmBeamSearch: public SearchAlgorithm{
public:

    //int maxNeighbourExplore = 1000;

    int maxLayerNodeExplore = 10000;

    double deformation_prune_ = std::numeric_limits<double>::max();


    SearchAlgorithmBeamSearch(std::shared_ptr<StateGraph> stateGraph,
                              //int maxNeighbourExplore,
                              int maxLayerNodeExplore,
                              double deformation_prune):
          SearchAlgorithm(stateGraph),
          maxLayerNodeExplore(maxLayerNodeExplore),
          deformation_prune_(deformation_prune)
          //maxNeighbourExplore(maxNeighbourExplore)
    {

    }

public:

    std::vector<HeapNode> candidates_;
    std::unordered_map<int, int> searchNodeInQueue_;

public:

    double search(AssemblySequence &sequence) override;

    double search(PtrS searchNode, AssemblySequence &sequence);

    void updateNode(PtrS currNode, PtrS updateNode);

    void expendNode(PtrS searchNode, std::vector<PtrS> &childNodeIndex) override;

};
}


#endif  //ROBO_CRAFT_SEARCHALGORITHMBEAMSEARCH_H
