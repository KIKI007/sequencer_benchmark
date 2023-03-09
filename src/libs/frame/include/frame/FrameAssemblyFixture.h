//
// Created by 汪子琦 on 15.09.22.
//

#ifndef ROBO_CRAFT_FRAMEASSEMBLYFIXTURE_H
#define ROBO_CRAFT_FRAMEASSEMBLYFIXTURE_H
#include "FrameAssembly.h"
namespace frame
{

class FrameAssemblyFixture: public frame::FrameAssembly{
public:

    const double fixture_stiffness_ = 1E3;

public:
    FrameAssemblyFixture(){}

    FrameAssemblyFixture(const FrameAssembly& assembly);

public:

    void computeFixtureStiffnessMatrix(const std::vector<int> &subset_beams_index,
                                       int total_dofs,
                                       const std::unordered_map<int, int> &map_dof_entire2subset,
                                       SparseMatrixD &K_fix);

    void solveElasticity(const std::vector<int> &subset_beams_index, Eigen::VectorXd &displacement);

};
}
#endif  //ROBO_CRAFT_FRAMEASSEMBLYFIXTURE_H
