//
// Created by 汪子琦 on 15.09.22.
//

#include "frame/FrameAssemblyFixture.h"

namespace frame {

FrameAssemblyFixture::FrameAssemblyFixture(const FrameAssembly &assembly) : FrameAssembly(assembly) {

}

void FrameAssemblyFixture::computeFixtureStiffnessMatrix(const std::vector<int> &subset_beams_index, int total_dofs,
                                                         const std::unordered_map<int, int> &map_dof_entire2subset, FrameAssembly::SparseMatrixD &K_fix) {
    K_fix = SparseMatrixD(total_dofs, total_dofs);
    K_fix.setZero();

    for (int id = 0; id < subset_beams_index.size(); id++) {
        int beamID = subset_beams_index[id];
        for (int edgeID = 0; edgeID < 2; edgeID++) {
            int vID = edges_[beamID][edgeID];
            for (int jd = 0; jd < 6; jd++) {
                int old_dof = vID * 6 + jd;
                auto find_it = map_dof_entire2subset.find(old_dof);
                if (find_it != map_dof_entire2subset.end()) {
                    int new_dof = find_it->second;
                    K_fix.coeffRef(new_dof, new_dof) += fixture_stiffness_ * beams_[beamID]->beam_fixture_;
                }
            }
        }
    }
    return;
}

void FrameAssemblyFixture::solveElasticity(const std::vector<int> &subset_beams_index, Eigen::VectorXd &displacement) {
    std::unordered_map<int, int> map_dof_entire2subset;
    std::unordered_map<int, int> map_dof_subset2entire;
    int total_dofs = computeDoFsMapping(subset_beams_index, {}, map_dof_entire2subset, map_dof_subset2entire);

    SparseMatrixD K_rho;
    computeStiffMatrix(subset_beams_index, total_dofs, map_dof_entire2subset, map_dof_subset2entire, K_rho);

    SparseMatrixD K_fix;
    computeFixtureStiffnessMatrix(subset_beams_index, total_dofs, map_dof_entire2subset, K_fix);

    Eigen::VectorXd F;
    computeLoads(subset_beams_index, total_dofs, map_dof_entire2subset, map_dof_subset2entire, F);

    Eigen::VectorXd D;

    SparseMatrixD K = K_rho + K_fix;

    Eigen::SimplicialLLT<SparseMatrixD> llt(K);
    if (llt.info() == Eigen::Success) {
        D = llt.solve(F);
        displacement = Eigen::VectorXd::Zero(vertices_.size() * 6);
        for (int id = 0; id < D.rows(); id++) {
            int old_dof = map_dof_subset2entire[id];
            displacement[old_dof] = D[id];
        }
    }
}
}
