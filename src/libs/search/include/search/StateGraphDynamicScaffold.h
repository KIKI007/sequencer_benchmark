//
// Created by 汪子琦 on 04.09.22.
//

#ifndef ROBO_CRAFT_STATEGRAPHDYNAMICSCAFFOLD_H
#define ROBO_CRAFT_STATEGRAPHDYNAMICSCAFFOLD_H
#include "StateGraph.h"
namespace search
{
class StateGraphDynamicScaffold : public StateGraph {
public:

    enum NodeStatus{
        None = 0,
        Install = 1,
        Fix = 2
    };

public:

    int numArms_;

    int numFixtures_;

    bool simplifyScaffoldSpace = false;
    bool edgeCost = false;

    std::vector<int> startPartIDs_;
    std::vector<int> startPartIDsFixed_;

    std::vector<int> endPartIDs_;
    std::vector<int> endPartIDsFixed_;

public:

    StateGraphDynamicScaffold(std::shared_ptr<PartGraph> graph, int numArms, int numFixtures): StateGraph(graph, 4)
    {
        numArms_ = numArms;
        numFixtures_ = numFixtures;
        startPartIDs_ = {};
        for(int id = 0; id < graph->nNode(); id++){
            endPartIDs_.push_back(id);
        }
    }

public:

    void createRootNode() override;

    void createEndNode(std::vector<unsigned> &state);

    bool checkEndNode(PtrN node) override;

    bool checkEndNode(std::vector<unsigned> &state);

    bool checkFixedPartinStartnEndFixed(int partID);

    void evaluateNode(PtrN node) override;

    double evaluateNode2Node(PtrN nA, PtrN nB) override;

    double evaluateNode2Node(PtrN nA, PtrN nB, std::vector<std::vector<int>> &fixedPartIDs);

    double evaluateNode2Node(const std::vector<int> &partA,
                             const std::vector<int> &partB,
                             const std::vector<int> &scaffoldA,
                             const std::vector<int> &scaffoldB,
                             AssemblySequence &sequence);

    double evaluateNode2Node(const std::vector<int> &partA,
                           const std::vector<int> &partB,
                           const std::vector<int> &scaffoldA,
                           const std::vector<int> &scaffoldB,
                           std::vector<std::vector<int>> &fixedPartIDs);

    void difference(const std::vector<int> &listA,
                    const std::vector<int> &listB,
                    std::vector<int> &listC);

    void unionSet(const std::vector<int> &listA,
                  const std::vector<int> &listB,
                  std::vector<int> &listC);

    void expandNode(PtrN node,
                    std::vector<PtrN> &preChildNodes,
                    std::vector<PtrN> &newChildNodes) override;

    void expandNodeWithoutScaffold(std::vector<unsigned> node,
                                   int remainArms,
                                   std::map<std::vector<unsigned>, bool, StateCompare> &nodeIncluded,
                                   std::vector<std::vector<unsigned >> &assemblyStates);

    void addNodes(std::vector<std::vector<unsigned >> &assemblyStates,
                  std::vector<PtrN> &preChildNodes,
                  std::vector<PtrN> &newChildNodes);

    void getSolution(std::vector<PtrN> nodes, AssemblySequence &sequence) override;

    void enumerateScaffold(const std::vector<int> &installPartList,
                           const std::vector<int> &fixedPartsList,
                           std::vector<std::vector<unsigned >> &states);

    void enumerateScaffold(const std::vector<int> &installPartList,
                           const std::vector<int> &fixedPartsList,
                           std::vector<int> fixedParts,
                           std::vector<std::vector<unsigned >> &states);


    std::vector<int> getFixedParts(PtrN node);

    std::vector<int> getFixedParts(const std::vector<unsigned > &state);

    std::string node_label(PtrN node) override;

    void enumerateNextAssemblyStates(int preStepOffset, int currStepOffset) override;

private:

    double getReleaseNode(const std::vector<int> &partIDs,
                          const std::vector<int> &fixedPartIDs,
                          const std::vector<int> &releasePartIDs,
                          int &resultID);

    double getInstallNode(const std::vector<int> &partIDs,
                          const std::vector<int> &fixedPartIDs,
                          const std::vector<int> &installPartIDs,
                          int &resultID);
};
}



#endif  //ROBO_CRAFT_STATEGRAPHDYNAMICSCAFFOLD_H
