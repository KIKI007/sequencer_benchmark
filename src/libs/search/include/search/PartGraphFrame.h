//
// Created by 汪子琦 on 04.09.22.
//

#ifndef ROBO_CRAFT_PARTGRAPHFRAME_H
#define ROBO_CRAFT_PARTGRAPHFRAME_H
#include "PartGraph.h"
#include "frame/FrameAssembly.h"

namespace search
{
class PartGraphFrameNode: public PartGraphNode{
public:
    bool beam_small;
    Eigen::Vector3d pu;
    Eigen::Vector3d pv;
};

class PartGraphFrameEdge: public PartGraphEdge{

};

class PartGraphFrame: public PartGraph{
public:

    std::shared_ptr<frame::FrameAssembly> assembly_;

    bool use_displacement_ = false;


public:

    PartGraphFrame(const PartGraphFrame &graph);

    PartGraphFrame(std::shared_ptr<frame::FrameAssembly> assembly);

public:

    void computeGraph();

public:

    double evaluateStability(const std::vector<int> &subPartIDs, const std::vector<int> &dynamic_scaffold_partIDs) override;

};

}


#endif  //ROBO_CRAFT_PARTGRAPHFRAME_H
