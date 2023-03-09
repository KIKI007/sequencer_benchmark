//
// Created by 汪子琦 on 01.12.22.
//
#include "dto/FrameComplianceDiff.h"

double dto::FrameComplianceDiff::computeSIMP(double rho, double ratio)
{
    return ratio + (1 - ratio) * rho;
}

double dto::FrameComplianceDiff::computeSIMPGradient(double rho, double ratio)
{
    return 1 - ratio;
}

void dto::FrameComplianceDiff::computeStiffnessMatrix(const Eigen::VectorXd& rho, Eigen::SparseMatrix<double>& K_stiff)
{
    std::vector<Eigen::Triplet<double>> tripleLists;
    for (int id = 0; id < assembly_->beams_elasticity_.size(); id++)
    {
        Eigen::MatrixXd k = beamStiff[id];
        Eigen::MatrixXd k_e = k * computeSIMP(rho[id], young_module_ratio_);
        assembleStiffMatrix(tripleLists, k_e, edge_dof_local2global[id]);
    }
    K_stiff = SparseMatrixD(total_dofs, total_dofs);
    K_stiff.setFromTriplets(tripleLists.begin(), tripleLists.end());
}

void dto::FrameComplianceDiff::computeForce(const Eigen::VectorXd& rho, Eigen::VectorXd& F)
{
    F = Eigen::VectorXd(total_dofs);
    F.setZero();

    for (int id = 0; id < assembly_->beams_elasticity_.size(); id++) {
        Eigen::VectorXd g = beamWeight[id];
        Eigen::VectorXd g_e = computeSIMP(rho[id], weight_ratio_) * g;
        assemblyForce(F, g_e, edge_dof_local2global[id]);
    }
}

double dto::FrameComplianceDiff::computeCompliance(const Eigen::VectorXd& rho)
{
    SparseMatrixD K_rho;
    computeStiffnessMatrix(rho, K_rho);

    Eigen::VectorXd g;
    computeForce(rho, g);

    Eigen::SimplicialLLT<SparseMatrixD> llt(K_rho);
    if(llt.info() == Eigen::Success){
        Eigen::VectorXd u = llt.solve(g);
        return 0.5 * g.dot(u);
    }
    return std::numeric_limits<double>::max();
}

double  dto::FrameComplianceDiff::computeMaxDisplacement(const Eigen::VectorXd &rho){
    SparseMatrixD K_rho;
    computeStiffnessMatrix(rho, K_rho);

    Eigen::VectorXd g;
    computeForce(rho, g);

    Eigen::SimplicialLLT<SparseMatrixD> llt(K_rho);
    if(llt.info() == Eigen::Success)
    {
        double max_disp = 0.0;
        Eigen::VectorXd u = llt.solve(g);
        for(int id = 0; id < u.size() / 6; id++){
            double disp = u.segment(id * 6, 3).norm();
            max_disp = std::max(disp, max_disp);
        }
        return max_disp;
    }
    return std::numeric_limits<double>::max();
}


double dto::FrameComplianceDiff::computeComplianceDerivatives(const Eigen::VectorXd& rho, const Eigen::VectorXd& u, Eigen::VectorXd& gradient)
{
    Eigen::VectorXd g;
    computeForce(rho, g);

    Eigen::VectorXd u_all = Eigen::VectorXd::Zero(assembly_->vertices_.size() * 6);
    for (int id = 0; id < u.rows(); id++) {
        int old_dof = map_dof_subset2entire[id];
        u_all[old_dof] = u[id];
    }

    gradient = Eigen::VectorXd(dynamic_partIDs_.size());
    gradient.setZero();

    for (int id = 0; id < dynamic_partIDs_.size(); id++)
    {
        int edgeID = dynamic_partIDs_[id];
        std::vector<int> dofs = assembly_->edgeDofIndices[edgeID];
        Eigen::VectorXd u_edge(dofs.size());
        for (int jd = 0; jd < dofs.size(); jd++) {
            u_edge(jd) = u_all[dofs[jd]];
        }

        Eigen::MatrixXd k_G = beamStiff[edgeID];
        Eigen::MatrixXd dK_G = computeSIMPGradient(rho[edgeID], young_module_ratio_) * k_G;

        Eigen::VectorXd w = beamWeight[edgeID];
        Eigen::VectorXd dw = computeSIMPGradient(rho[edgeID], weight_ratio_) * w;
        gradient(id) = 2 * dw.dot(u_edge) - u_edge.dot(dK_G * u_edge);
    }

    return g.dot(u);
}

void dto::FrameComplianceDiff::computeComplianceHessian(const Eigen::VectorXd& rho, Eigen::MatrixXd& hessian)
{
    Eigen::VectorXd F;
    SparseMatrixD K;
    computeStiffnessMatrix(rho, K);
    computeForce(rho, F);

    Eigen::SimplicialLLT<SparseMatrixD> llt(K);
    if(llt.info() == Eigen::Success)
    {
        Eigen::VectorXd u = llt.solve(F);
        Eigen::MatrixXd dF_dKu;
        compute_dF_dKu(u, rho, dF_dKu);
        hessian = 2.0 * dF_dKu.transpose() * llt.solve(dF_dKu);
    }
}

void dto::FrameComplianceDiff::setup()
{
    computeDoFMaps();
    computeEdgeLocal2GlobalDof();
    computeBeamStiffnWeight();
}

void dto::FrameComplianceDiff::computeDoFMaps()
{
    std::vector<int> subset_beams;
    for (int id = 0; id < assembly_->edges_.size(); id++) {
        subset_beams.push_back(id);
    }

    map_dof_entire2subset.clear();
    map_dof_subset2entire.clear();
    total_dofs = assembly_->computeDoFsMapping(subset_beams, {}, map_dof_entire2subset, map_dof_subset2entire);
}

void dto::FrameComplianceDiff::computeEdgeLocal2GlobalDof()
{
    edge_dof_local2global.clear();
    for (int edgeID = 0; edgeID < assembly_->edgeDofIndices.size(); edgeID++)
    {
        std::vector<int> dofs;
        for (int jd = 0; jd < assembly_->edgeDofIndices[edgeID].size(); jd++)
        {
            int old_dof = assembly_->edgeDofIndices[edgeID][jd];
            if (map_dof_entire2subset.find(old_dof) != map_dof_entire2subset.end()) {
                dofs.push_back(map_dof_entire2subset.at(old_dof));
            } else {
                dofs.push_back(-1);
            }
        }

        edge_dof_local2global.push_back(dofs);
    }
}

void dto::FrameComplianceDiff::computeBeamStiffnWeight() {
    beamStiff.clear();
    beamWeight.clear();
    for (int edgeID = 0; edgeID < assembly_->beams_elasticity_.size(); edgeID++)
    {
        beamStiff.push_back(assembly_->beams_elasticity_[edgeID]->create_global_stiffness_matrix(true));
        beamWeight.push_back(assembly_->beams_elasticity_[edgeID]->create_global_self_weight());
    }
}

void dto::FrameComplianceDiff::assembleStiffMatrix(std::vector<Eigen::Triplet<double>>& K_tri, const Eigen::MatrixXd& k_G,
                                                   std::vector<int>& dofs_local2global) {
    for (int jd = 0; jd < 12; jd++)
    {
        for (int kd = 0; kd < 12; kd++)
        {
            //if(abs(k_G(jd, kd)) < 1E-12) continue;
            int new_dofJ = dofs_local2global[jd];
            int new_dofK = dofs_local2global[kd];
            if (new_dofJ != -1 && new_dofK != -1) {
                K_tri.push_back(Eigen::Triplet<double>(new_dofJ, new_dofK, k_G(jd, kd)));
            }
        }
    }
}

void dto::FrameComplianceDiff::assemblyForce(Eigen::VectorXd& F, const Eigen::VectorXd& g, std::vector<int>& dofs_local2global)
{
    for (int jd = 0; jd < dofs_local2global.size(); jd++)
    {
        int new_dofJ = dofs_local2global[jd];
        if (new_dofJ != -1) {
            F(new_dofJ) += g[jd];
        }
    }
}

void dto::FrameComplianceDiff::compute_dF_dKu(const Eigen::VectorXd& u, const Eigen::VectorXd& rho, Eigen::MatrixXd& dF_dKu) {
    Eigen::VectorXd u_all = Eigen::VectorXd::Zero(assembly_->vertices_.size() * 6);
    for (int id = 0; id < u.rows(); id++)
    {
        int old_dof = map_dof_subset2entire[id];
        u_all[old_dof] = u[id];
    }

    dF_dKu = Eigen::MatrixXd(total_dofs, dynamic_partIDs_.size());
    dF_dKu.setZero();

    for (int id = 0; id < dynamic_partIDs_.size(); id++)
    {
        int edgeID = dynamic_partIDs_[id];
        std::vector<int> dofs = assembly_->edgeDofIndices[edgeID];
        Eigen::VectorXd u_edge(dofs.size());
        for (int jd = 0; jd < dofs.size(); jd++) {
            u_edge(jd) = u_all[dofs[jd]];
        }

        Eigen::MatrixXd k_G = beamStiff[edgeID];
        Eigen::MatrixXd dK_G = computeSIMPGradient(rho[id], young_module_ratio_) * k_G;

        Eigen::MatrixXd w_G = beamWeight[edgeID];
        Eigen::MatrixXd dw_G = computeSIMPGradient(rho[id], weight_ratio_) * w_G;

        Eigen::VectorXd df_dKu = dw_G - dK_G * u_edge;

        for(int jd = 0; jd < dofs.size(); jd++)
        {
            int icol = id;
            if(map_dof_entire2subset.find(dofs[jd]) != map_dof_entire2subset.end()){
                int irow = map_dof_entire2subset[dofs[jd]];
                dF_dKu(irow, icol) += df_dKu[jd];
            }
        }
    }
}

bool dto::FrameComplianceDiff::computeDisplacement(const Eigen::VectorXd& rho, Eigen::VectorXd& u) {
    //force and stiffness
    Eigen::VectorXd F;
    SparseMatrixD K;
    computeStiffnessMatrix(rho, K);
    computeForce(rho, F);

    Eigen::SimplicialLLT<SparseMatrixD> llt(K);

    if (llt.info() == Eigen::Success) {
        u = llt.solve(F);
        return true;
    } else {
        return false;
    }
}
