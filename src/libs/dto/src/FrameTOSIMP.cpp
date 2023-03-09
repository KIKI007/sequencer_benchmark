//
// Created by 汪子琦 on 03.05.22.
//

#include "dto/FrameTOSIMP.h"
#include <cmath>
namespace dto
{

FrameTOSIMP::FrameTOSIMP(const FrameTOSIMP &frameTo) {
    simp_penlty_ = frameTo.simp_penlty_;
    weight_ratio_ = frameTo.weight_ratio_;
    young_module_ratio_ = frameTo.young_module_ratio_;
    numTargetBeam_ = frameTo.numTargetBeam_;
    assembly_ = frameTo.assembly_;
    external_force_ = frameTo.external_force_;
    budget_equal = frameTo.budget_equal;
}

double FrameTOSIMP::computeSIMP(double rho, double ratio, int power)
{
    double base = 1;
    for(int id = 0; id < power; id++)
    {
        base *= rho;
    }
    return ratio + (1 - ratio) * base;
}

double FrameTOSIMP::computeSIMPGradient(double rho, double ratio,  int power) {

    if(power >= 1)
    {
        double base = 1;
        for(int id = 1; id < power; id++)
        {
            base *= rho;
        }

        return (1 - ratio) * power * base;
    }

    return 0.0;
}

double FrameTOSIMP::computeSIMPHessian(double rho, double ratio, int power) {

    if(power >= 2)
    {
        double base = 1;
        for(int id = 2; id < power; id++)
        {
            base *= rho;
        }

        return (1 - ratio) * power * (power - 1) * base;
    }

    return 0;
}

void FrameTOSIMP::assembleStiffMatrix(std::vector<Eigen::Triplet<double>> &K_tri, const Eigen::MatrixXd &k_G, std::vector<int> &dofs_local2global) {
    for (int jd = 0; jd < 12; jd++) {
        for (int kd = 0; kd < 12; kd++) {
            //if(abs(k_G(jd, kd)) < 1E-12) continue;
            int new_dofJ = dofs_local2global[jd];
            int new_dofK = dofs_local2global[kd];
            if (new_dofJ != -1 && new_dofK != -1)
            {
                K_tri.push_back(Eigen::Triplet<double>(new_dofJ, new_dofK, k_G(jd, kd)));
            }
        }
    }
}

void FrameTOSIMP::assemblyForce(Eigen::VectorXd &F, const Eigen::VectorXd &g, std::vector<int> &dofs_local2global)
{
    for (int jd = 0; jd < dofs_local2global.size(); jd++) {
        int new_dofJ = dofs_local2global[jd];
        if (new_dofJ != -1) {
            F(new_dofJ) += g[jd];
        }
    }
}

void FrameTOSIMP::setup()
{
    computeDoFMaps();
    computeEdgeLocal2GlobalDof();
    computeBeamStiffnWeight();
    computeAdjacentBeams();
}

void FrameTOSIMP::computeEdgeLocal2GlobalDof()
{
    edge_dof_local2global.clear();
    for (int edgeID = 0; edgeID < assembly_->edgeDofIndices.size(); edgeID++) {
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

void FrameTOSIMP::computeBeamStiffnWeight()
{
    beamStiff.clear();
    beamWeight.clear();
    for (int edgeID = 0; edgeID < assembly_->beams_elasticity_.size(); edgeID++)
    {
        beamStiff.push_back(assembly_->beams_elasticity_[edgeID]->create_global_stiffness_matrix());
        beamWeight.push_back(assembly_->beams_elasticity_[edgeID]->create_global_self_weight());
    }
}

void FrameTOSIMP::computeDoFMaps()
{
    std::vector<int> subset_beams;
    for (int id = 0; id < assembly_->edges_.size(); id++) {
        subset_beams.push_back(id);
    }

    map_dof_entire2subset.clear();
    map_dof_subset2entire.clear();

    total_dofs = assembly_->computeDoFsMapping(subset_beams, {},  map_dof_entire2subset, map_dof_subset2entire);
}

double FrameTOSIMP::computeComplianceDerivatives(const Eigen::VectorXd &rho,
                                                 const Eigen::VectorXd &u,
                                                 Eigen::VectorXd &gradient) {
    double obj = 0;
    Eigen::VectorXd u_all = Eigen::VectorXd::Zero(assembly_->vertices_.size() * 6);
    for (int id = 0; id < u.rows(); id++) {
        int old_dof = map_dof_subset2entire[id];
        u_all[old_dof] = u[id];
    }

    gradient = Eigen::VectorXd(rho.size());
    gradient.setZero();

    for (int id = 0; id < assembly_->beams_elasticity_.size(); id++)
    {
        int edgeID = id;
        std::vector<int> dofs = assembly_->edgeDofIndices[edgeID];
        Eigen::VectorXd u_edge(dofs.size());
        for (int jd = 0; jd < dofs.size(); jd++) {
            u_edge(jd) = u_all[dofs[jd]];
        }

        Eigen::MatrixXd k_G = beamStiff[edgeID];
        Eigen::MatrixXd dK_G = computeSIMPGradient(rho[id], young_module_ratio_, simp_penlty_) * k_G;

        Eigen::VectorXd w = beamWeight[edgeID];
        Eigen::VectorXd dw = computeSIMPGradient(rho[id], weight_ratio_, 1) * w;
        gradient(id) = -u_edge.dot(dK_G * u_edge) + 2 * dw.dot(u_edge);
    }

    Eigen::VectorXd F;
    computeForce(rho, F);
    if(external_force_.size() == total_dofs){
        F += external_force_;
    }
    obj = F.dot(u);

    return obj;
}

void FrameTOSIMP::computeComplianceHessianVector(const Eigen::VectorXd &rho,
                                                 const Eigen::VectorXd &vec,
                                                 double *hessian) {
    Eigen::VectorXd result(nX());
    result.setZero();

    Eigen::VectorXd F;
    SparseMatrixD K;
    computeStiffnessMatrix(rho, K);
    computeForce(rho, F);

    //external Force
    if(external_force_.size() == total_dofs){
        F += external_force_;
    }

    Eigen::SimplicialLLT<SparseMatrixD> llt(K);

    if(llt.info() == Eigen::Success)
    {
        Eigen::VectorXd u = llt.solve(F);

        Eigen::VectorXd u_all = Eigen::VectorXd::Zero(assembly_->vertices_.size() * 6);
        for (int id = 0; id < u.rows(); id++)
        {
            int old_dof = map_dof_subset2entire[id];
            u_all[old_dof] = u[id];
        }

        std::vector<Eigen::Triplet<double>> df_dKu_triList;

        for (int id = 0; id < assembly_->beams_elasticity_.size(); id++)
        {
            int edgeID = id;
            std::vector<int> dofs = assembly_->edgeDofIndices[edgeID];
            Eigen::VectorXd u_edge(dofs.size());
            for (int jd = 0; jd < dofs.size(); jd++) {
                u_edge(jd) = u_all[dofs[jd]];
            }

            Eigen::MatrixXd k_G = beamStiff[edgeID];
            Eigen::MatrixXd dK_G = computeSIMPGradient(rho[id], young_module_ratio_, simp_penlty_) * k_G;
            Eigen::MatrixXd d2K_G = computeSIMPHessian(rho[id], young_module_ratio_, simp_penlty_) * k_G;

            Eigen::MatrixXd w_G = beamWeight[edgeID];
            Eigen::MatrixXd dw_G = computeSIMPGradient(rho[id], weight_ratio_, 1) * w_G;

            Eigen::VectorXd df_dKu = dw_G - dK_G * u_edge;

            for(int jd = 0; jd < dofs.size(); jd++){
                int icol = id;
                if(map_dof_entire2subset.find(dofs[jd]) != map_dof_entire2subset.end()){
                    int irow = map_dof_entire2subset[dofs[jd]];
                    df_dKu_triList.push_back({irow, icol, df_dKu[jd]});
                }
            }

            if(simp_penlty_ > 1) {
                result[id] += -u_edge.dot(d2K_G * u_edge) * vec[id];
            }
        }

        SparseMatrixD df_dKu(total_dofs, nX());
        df_dKu.setFromTriplets(df_dKu_triList.begin(), df_dKu_triList.end());
        Eigen::VectorXd df_dKu_vec = df_dKu * vec;
        result += 2 * df_dKu.transpose() * (llt.solve(df_dKu_vec));
    }

    for(int id = 0; id < nX(); id++){
        hessian[id] = result[id];
    }
}

void FrameTOSIMP::computeComplianceHessian(const Eigen::VectorXd &rho, Eigen::MatrixXd &hessian)
{
    Eigen::VectorXd F;
    SparseMatrixD K;
    computeStiffnessMatrix(rho, K);
    computeForce(rho, F);

    //external Force
    if(external_force_.size() == total_dofs){
        F += external_force_;
    }

    hessian = Eigen::MatrixXd(rho.size(), rho.size());
    hessian.setZero();

    Eigen::SimplicialLLT<SparseMatrixD> llt(K);

    if(llt.info() == Eigen::Success)
    {
        Eigen::VectorXd u = llt.solve(F);

        Eigen::VectorXd u_all = Eigen::VectorXd::Zero(assembly_->vertices_.size() * 6);
        for (int id = 0; id < u.rows(); id++)
        {
            int old_dof = map_dof_subset2entire[id];
            u_all[old_dof] = u[id];
        }

        std::vector<Eigen::Triplet<double>> df_dKu_triList;

        for (int id = 0; id < assembly_->beams_elasticity_.size(); id++)
        {
            int edgeID = id;
            std::vector<int> dofs = assembly_->edgeDofIndices[edgeID];
            Eigen::VectorXd u_edge(dofs.size());
            for (int jd = 0; jd < dofs.size(); jd++) {
                u_edge(jd) = u_all[dofs[jd]];
            }

            Eigen::MatrixXd k_G = beamStiff[edgeID];
            Eigen::MatrixXd dK_G = computeSIMPGradient(rho[id], young_module_ratio_, simp_penlty_) * k_G;
            Eigen::MatrixXd d2K_G = computeSIMPHessian(rho[id], young_module_ratio_, simp_penlty_) * k_G;

            Eigen::MatrixXd w_G = beamWeight[edgeID];
            Eigen::MatrixXd dw_G = computeSIMPGradient(rho[id], weight_ratio_, 1) * w_G;

            Eigen::VectorXd df_dKu = dw_G - dK_G * u_edge;

            for(int jd = 0; jd < dofs.size(); jd++){
                int icol = id;
                if(map_dof_entire2subset.find(dofs[jd]) != map_dof_entire2subset.end()){
                    int irow = map_dof_entire2subset[dofs[jd]];
                    df_dKu_triList.push_back({irow, icol, df_dKu[jd]});
                }
            }

            if(simp_penlty_ > 1)
            {
                hessian(id, id) += -u_edge.dot(d2K_G * u_edge);
            }
        }

        SparseMatrixD df_dKu(total_dofs, nX());
        df_dKu.setFromTriplets(df_dKu_triList.begin(), df_dKu_triList.end());

        Eigen::MatrixXd Kinv_df_dKu = llt.solve(df_dKu);
        hessian += 2 * df_dKu.transpose() * Kinv_df_dKu;
    }
}

void FrameTOSIMP::computeComplianceHessian(const Eigen::VectorXd &rho,
                                           const Eigen::VectorXd &u,
                                           const Eigen::MatrixXd &du,
                                           SparseMatrixD &K,
                                           Eigen::MatrixXd &hessian)
{
    hessian = (2 * du.transpose() * (K * du));

    if(simp_penlty_ > 1)
    {
        Eigen::VectorXd u_all = Eigen::VectorXd::Zero(assembly_->vertices_.size() * 6);
        for (int id = 0; id < u.rows(); id++) {
            int old_dof = map_dof_subset2entire[id];
            u_all[old_dof] = u[id];
        }

        for (int id = 0; id < assembly_->beams_elasticity_.size(); id++)
        {
            int edgeID = id;
            std::vector<int> dofs = assembly_->edgeDofIndices[edgeID];
            Eigen::VectorXd u_edge(dofs.size());
            for (int jd = 0; jd < dofs.size(); jd++) {
                u_edge(jd) = u_all[dofs[jd]];
            }

            Eigen::MatrixXd k_G = beamStiff[edgeID];
            Eigen::MatrixXd d2K_G = computeSIMPHessian(rho[id], young_module_ratio_, simp_penlty_) * k_G;
            hessian(id, id) += -u_edge.dot(d2K_G * u_edge);
        }
    }
}

void FrameTOSIMP::computeForce(const Eigen::VectorXd &rho, Eigen::VectorXd &F) {
    F = Eigen::VectorXd(total_dofs);
    F.setZero();

    for (int id = 0; id < assembly_->beams_elasticity_.size(); id++) {
        Eigen::VectorXd g = beamWeight[id];
        Eigen::VectorXd g_e = computeSIMP(rho[id], weight_ratio_, 1) * g;
        assemblyForce(F, g_e, edge_dof_local2global[id]);
    }
}

void FrameTOSIMP::computeForceDerivatives(const Eigen::VectorXd &rho, std::vector<Eigen::VectorXd> &dF) {
    for (int id = 0; id < assembly_->beams_elasticity_.size(); id++) {
        Eigen::VectorXd dF_i = Eigen::VectorXd(total_dofs);
        dF_i.setZero();
        Eigen::VectorXd g = beamWeight[id];
        Eigen::VectorXd dg = computeSIMPGradient(rho[id], weight_ratio_, 1) * g;
        assemblyForce(dF_i, dg, edge_dof_local2global[id]);
        dF.push_back(dF_i);
    }
}

void FrameTOSIMP::computeStiffnessMatrix(const Eigen::VectorXd &rho, SparseMatrixD &K_stiff) {
    std::vector<Eigen::Triplet<double>> tripleLists;
    for (int id = 0; id < assembly_->beams_elasticity_.size(); id++) {
        Eigen::MatrixXd k = beamStiff[id];
        Eigen::MatrixXd k_e = computeSIMP(rho[id], young_module_ratio_, simp_penlty_) * k;
        assembleStiffMatrix(tripleLists, k_e, edge_dof_local2global[id]);
    }
    K_stiff = SparseMatrixD(total_dofs, total_dofs);
    K_stiff.setFromTriplets(tripleLists.begin(), tripleLists.end());
}

void FrameTOSIMP::computeStiffnessMatrixDerivatives(const Eigen::VectorXd &rho, std::vector<SparseMatrixD> &dK) {
    for (int id = 0; id < assembly_->beams_elasticity_.size(); id++) {
        std::vector<Eigen::Triplet<double>> tripleLists;
        Eigen::MatrixXd k = beamStiff[id];
        Eigen::MatrixXd dk = computeSIMPGradient(rho[id], young_module_ratio_, simp_penlty_) * k;
        assembleStiffMatrix(tripleLists, dk, edge_dof_local2global[id]);
        SparseMatrixD dK_stiff = SparseMatrixD(total_dofs, total_dofs);
        dK_stiff.setFromTriplets(tripleLists.begin(), tripleLists.end());
        dK.push_back(dK_stiff);
    }
}

bool FrameTOSIMP::computeDisplacement(const Eigen::VectorXd &rho, Eigen::VectorXd &u) {
    //force and stiffness
    Eigen::VectorXd F;
    SparseMatrixD K;
    computeStiffnessMatrix(rho, K);
    computeForce(rho, F);

    //external Force
    if(external_force_.size() == total_dofs){
        F += external_force_;
    }

    //
    Eigen::SimplicialLLT<SparseMatrixD> llt(K);
    if (llt.info() == Eigen::Success)
    {
        u = llt.solve(F);
        return true;
    }
    return false;
}


bool FrameTOSIMP::computeDisplacementDerivatives(const Eigen::VectorXd &rho, Eigen::VectorXd &u, Eigen::MatrixXd &du, SparseMatrixD &K) {
    //force and stiffness
    Eigen::VectorXd F;
    K.setZero();
    computeStiffnessMatrix(rho, K);
    computeForce(rho, F);

    //external Force
    if(external_force_.size() == total_dofs){
        F += external_force_;
    }

    //
    Eigen::SimplicialLLT<SparseMatrixD> llt(K);
    du = Eigen::MatrixXd(F.rows(), rho.size());
    du.setZero();

    if (llt.info() == Eigen::Success)
    {
        u = llt.solve(F);

        std::vector<SparseMatrixD> dK;
        computeStiffnessMatrixDerivatives(rho, dK);

        std::vector<Eigen::VectorXd> dF;
        computeForceDerivatives(rho, dF);

        for (int id = 0; id < rho.size(); id++) {
            Eigen::VectorXd du_i;
            Eigen::VectorXd dK_iu = dK[id] * u;

            du_i = llt.solve(dF[id]) - llt.solve(dK_iu);
            du.col(id) = du_i;
        }
        return true;
    } else {
        return false;
    }
}

int FrameTOSIMP::nX() {
    return assembly_->beams_.size();
}

void FrameTOSIMP::getXTypes(std::vector<int> &x_types, bool MINLP) {
    if (MINLP)
        x_types.resize(nX(), KN_VARTYPE_BINARY);
    else
        x_types.resize(nX(), KN_VARTYPE_CONTINUOUS);
}

void FrameTOSIMP::initializeX(Eigen::VectorXd &x) {
    x = Eigen::VectorXd::Zero(nX());

    for (int id = 0; id < x.size(); id++) {
        x(id) = (double)numTargetBeam_ / x.size();
    }
}

void FrameTOSIMP::computeObjectiveGradient(const double *x, double *obj, double *obj_grad) {
    Eigen::VectorXd rho(nX());
    for (int id = 0; id < nX(); id++) {
        rho(id) = x[id];
    }

    Eigen::VectorXd u;
    computeDisplacement(rho, u);

    Eigen::VectorXd dobj_r;
    *obj = computeComplianceDerivatives(rho, u, dobj_r);

    for(int id = 0; id < heights_.size(); id++){
        *obj += weight_laplacian_ * rho[id] * heights_[id];
        dobj_r[id] += weight_laplacian_ * heights_[id];
    }

    for (int id = 0; id < dobj_r.size(); id++) {
        obj_grad[id] = dobj_r(id);
    }
}

void FrameTOSIMP::computeConstraintsGradient(const double *x, double *c, double *jac) {
    c[0] = -std::min(numTargetBeam_, nX());
    for (int id = 0; id < nX(); id++) {
        c[0] += x[id];
    }

    for (int id = 0; id < nX(); id++) {
        jac[id] = 1;
    }
}

void FrameTOSIMP::computeHessianVector(const double *x, const double *v, double sigma, const double *lambda, double *hess)
{
    Eigen::VectorXd rho(nX());
    Eigen::VectorXd vec(nX());
    for (int id = 0; id < nX(); id++) {
        rho(id) = x[id];
        vec(id) = v[id];
    }

    computeComplianceHessianVector(rho, vec, hess);
    for(int id = 0; id < nX(); id++){
        hess[id] *= sigma;
    }

    return ;
}

void FrameTOSIMP::computeHessian(const double *x, double sigma, const double *lambda, double *hess) {
    Eigen::VectorXd rho(nX());
    for (int id = 0; id < nX(); id++) {
        rho(id) = x[id];
    }

    Eigen::MatrixXd hessian;
    computeComplianceHessian(rho, hessian);

    std::vector<Eigen::Vector2i> index;
    computeHessIndex(index);
    for (int id = 0; id < index.size(); id++)
    {
        hess[id] = hessian(index[id].x(), index[id].y()) * sigma;
    }
}

void FrameTOSIMP::computeJacIndex(std::vector<Eigen::Vector2i> &index) {
    for (int id = 0; id < nX(); id++) {
        index.push_back(Eigen::Vector2i(0, id));
    }
}

void FrameTOSIMP::computeHessIndex(std::vector<Eigen::Vector2i> &index) {
    for (int id = 0; id < nX(); id++)
    {
        for (int jd = id; jd < nX(); jd++)
        {
            index.push_back(Eigen::Vector2i(id, jd));
        }
    }
}

void FrameTOSIMP::computeVariableBounds(std::vector<double> &xmin, std::vector<double> &xmax) {
    for (int id = 0; id < nX(); id++) {
        xmin.push_back(0);
        xmax.push_back(1);
    }
}
void FrameTOSIMP::computeConstraintsBound(std::vector<double> &cmin, std::vector<double> &cmax) {
    if(budget_equal){
        cmin.push_back(-0.01);
        cmax.push_back(0.01);
    }
    else{
        cmin.push_back(-KN_INFINITY);
        cmax.push_back(0.01);
    }
}

std::vector<double> FrameTOSIMP::evaluateAssembly(Eigen::VectorXd &rho) {
    std::vector<int> subset_beams;
    for (int id = 0; id < assembly_->edges_.size(); id++) {
        int edgeID = id;
        subset_beams.push_back(edgeID);
    }

    updateBeams(rho);

    Eigen::VectorXd displacement;
    assembly_->solveElasticity(subset_beams, {},  displacement);

    double compliance = assembly_->computeCompliance(displacement, subset_beams);
    double max_displacement = assembly_->computeMaxDisplacement(displacement);

    std::vector<Eigen::VectorXd> internal_forces;
    std::vector<Eigen::Vector2d> joint_stresses;
    double max_stress = assembly_->computeMaxStress(subset_beams, displacement, internal_forces, joint_stresses);

    return {compliance, max_displacement, max_stress};
}

void FrameTOSIMP::updateBeams(Eigen::VectorXd &rho) {
    assembly_->computeBeams();
    assembly_->computeBeamElasticity();

    for (int id = 0; id < assembly_->edges_.size(); id++) {
        int edgeID = id;
        double r = rho(id);
        assembly_->beams_[edgeID]->beam_width_ *= r;
        assembly_->beams_elasticity_[edgeID]->beam_width_ *= r;
        assembly_->beams_elasticity_[edgeID]->material_.E *= computeSIMP(r, young_module_ratio_, simp_penlty_);
        assembly_->beams_elasticity_[edgeID]->material_.rho *= computeSIMP(r, weight_ratio_, 1);
        assembly_->beams_elasticity_[edgeID]->material_.width = assembly_->beams_[edgeID]->beam_width_;
    }
}

void FrameTOSIMP::printSolution(const double *x)
{
    int nE = assembly_->beams_.size();
    std::cout << "node = {";
    for(int id = 0; id < nE; id++)
    {
        if(x[id] > 0.5){
            std::cout << id << ", ";
        }
    }
    std::cout << "};\n";
}

void FrameTOSIMP::setExternalForce(Eigen::VectorXd force)
{
    setup();
    external_force_ = Eigen::VectorXd(total_dofs);
    external_force_.setZero();
    for(int id = 0; id < force.size(); id++){
        int old_dof = id;
        auto find_it = map_dof_entire2subset.find(old_dof);
        if(find_it != map_dof_entire2subset.end()){
            external_force_[find_it->second] = force[id];
        }
    }
}
void FrameTOSIMP::computeAdjacentBeams()
{
    double maxY = 0;
    heights_.clear();
    for(int id = 0; id < assembly_->beams_.size(); id++){
        heights_.push_back(assembly_->beams_[id]->computeCenterY());
        maxY = std::max(maxY, heights_.back());
    }
    for(int id = 0; id < heights_.size(); id++){
        heights_[id] /= maxY;
    }
}

}