//
// Created by 汪子琦 on 14.09.22.
//

#include "dto/FrameTOSupport.h"

namespace dto
{

void FrameTOSupport::computeVars(const double *x, Eigen::VectorXd &rho, Eigen::VectorXd &fix)
{
    //rho
    rho = rho_base_;
    for(int id = 0; id < nE_rho(); id++)
    {
        int partID = dynamic_partIDs_[id];
        rho[partID] = x[id];
    }

    //fix
    fix = Eigen::VectorXd::Zero(nE());
    for (int id = 0; id < nE_fix(); id++)
    {
        int partID = end_partIDs_[id];
        fix[partID] = x[id + nE_rho()];
    }
}

void FrameTOSupport::computeFixtureStiffnessMatrix(const Eigen::VectorXd &fix, FrameTO::SparseMatrixD &K_fixture) {
    K_fixture = SparseMatrixD(total_dofs, total_dofs);
    std::vector<Eigen::Triplet<double>> triList;

    for (int id = 0; id < assembly_->edges_.size(); id++)
    {
        for (int jd = 0; jd < 2; jd++) {
            int vID = assembly_->edges_[id][jd];
            for (int kd = 0; kd < 6; kd++) {
                int old_dof = vID * 6 + kd;
                auto find_it = map_dof_entire2subset.find(old_dof);
                if (find_it != map_dof_entire2subset.end())
                {
                    int new_dof = map_dof_entire2subset[old_dof];
                    triList.push_back({new_dof, new_dof, fixture_stiffness_ * fix[id]});
                }
            }
        }
    }

    K_fixture.setFromTriplets(triList.begin(), triList.end());
    return;
}

double FrameTOSupport::computeComplianceDerivatives(const Eigen::VectorXd &rho,
                                                    const Eigen::VectorXd &fix,
                                                    const Eigen::VectorXd &u,
                                                    Eigen::VectorXd &gradient)
{
    Eigen::VectorXd g;
    computeForce(rho, g);

    double obj = g.dot(u);

    Eigen::VectorXd u_all = Eigen::VectorXd::Zero(assembly_->vertices_.size() * 6);
    for (int id = 0; id < u.rows(); id++) {
        int old_dof = map_dof_subset2entire[id];
        u_all[old_dof] = u[id];
    }

    gradient = Eigen::VectorXd(nX());
    gradient.setZero();

    auto get_u_edge = [&](int edgeID)
    {
        std::vector<int> dofs = assembly_->edgeDofIndices[edgeID];
        Eigen::VectorXd u_edge(dofs.size());
        for (int jd = 0; jd < dofs.size(); jd++) {
            u_edge(jd) = u_all[dofs[jd]];
        }
        return u_edge;
    };

    for (int id = 0; id < nE_rho(); id++)
    {
        int edgeID = dynamic_partIDs_[id];

        Eigen::VectorXd u_edge = get_u_edge(edgeID);

        Eigen::MatrixXd K_frame = beamStiff[edgeID];
        Eigen::MatrixXd dK_frame = computeSIMPGradient(rho[edgeID], young_module_ratio_) * K_frame;

        Eigen::VectorXd w = beamWeight[edgeID];
        Eigen::VectorXd dw_frame = computeSIMPGradient(rho[edgeID], weight_ratio_) * w;
        gradient(id) = -u_edge.dot(dK_frame * u_edge) + 2 * dw_frame.dot(u_edge);
    }

    for(int id = 0; id < nE_fix(); id++)
    {
        int edgeID = end_partIDs_[id];
        Eigen::VectorXd u_edge = get_u_edge(edgeID);
        gradient(id + nE_rho()) = -u_edge.dot(u_edge) * fixture_stiffness_;
    }

    return obj;
}

bool FrameTOSupport::computeDisplacement(const Eigen::VectorXd &rho, const Eigen::VectorXd &fix, Eigen::VectorXd &u) {
    SparseMatrixD K_rho;
    computeStiffnessMatrix(rho, K_rho);

    Eigen::VectorXd g;
    computeForce(rho, g);

    SparseMatrixD K_fix;
    computeFixtureStiffnessMatrix(fix, K_fix);

    SparseMatrixD K = K_fix + K_rho;

    Eigen::SimplicialLLT<SparseMatrixD> llt(K);
    if (llt.info() == Eigen::Success) {
        u = llt.solve(g);
        return true;
    } else {
        return false;
    }
}

void FrameTOSupport::computeObjectiveGradient(const double *x, double *obj, double *obj_gradient) {
    Eigen::VectorXd rho, fix;
    computeVars(x, rho, fix);

    Eigen::VectorXd u;

    if (computeDisplacement(rho, fix, u))
    {
        Eigen::VectorXd gradient;
        *obj = computeComplianceDerivatives(rho, fix, u, gradient);
        for (int id = 0; id < nX(); id++) {
            obj_gradient[id] = gradient[id];
        }
    } else {
        *obj = std::numeric_limits<double>::max();
    }
}

void FrameTOSupport::compute_dF_dKu(const Eigen::VectorXd &u, const Eigen::VectorXd &rho, const Eigen::VectorXd &fix, Eigen::MatrixXd &dF_dKu) {
    Eigen::VectorXd u_all = Eigen::VectorXd::Zero(assembly_->vertices_.size() * 6);
    for (int id = 0; id < u.rows(); id++)
    {
        int old_dof = map_dof_subset2entire[id];
        u_all[old_dof] = u[id];
    }

    dF_dKu = Eigen::MatrixXd(total_dofs, nX());
    dF_dKu.setZero();

    auto get_u_edge = [&](int edgeID)
    {
        std::vector<int> dofs = assembly_->edgeDofIndices[edgeID];
        Eigen::VectorXd u_edge(dofs.size());
        for (int jd = 0; jd < dofs.size(); jd++) {
            u_edge(jd) = u_all[dofs[jd]];
        }
        return u_edge;
    };

    for (int id = 0; id < nE_rho(); id++)
    {
        int edgeID = dynamic_partIDs_[id];
        Eigen::VectorXd u_edge = get_u_edge(edgeID);

        Eigen::MatrixXd k_G = beamStiff[edgeID];
        Eigen::MatrixXd dK_G = computeSIMPGradient(rho[id], young_module_ratio_) * k_G;

        Eigen::MatrixXd w_G = beamWeight[edgeID];
        Eigen::MatrixXd dw_G = computeSIMPGradient(rho[id], weight_ratio_) * w_G;

        Eigen::VectorXd df_dKu = dw_G - dK_G * u_edge;

        std::vector<int> dofs = assembly_->edgeDofIndices[edgeID];
        for(int jd = 0; jd < dofs.size(); jd++)
        {
            int varID = id;
            if(map_dof_entire2subset.find(dofs[jd]) != map_dof_entire2subset.end()){
                int irow = map_dof_entire2subset[dofs[jd]];
                dF_dKu(irow, varID) += df_dKu[jd];
            }
        }
    }

    for(int id = 0; id < nE_fix(); id++){
        int edgeID = end_partIDs_[id];

        Eigen::VectorXd u_edge = get_u_edge(edgeID);
        Eigen::VectorXd df_dKu = -fixture_stiffness_ * u_edge;

        std::vector<int> dofs = assembly_->edgeDofIndices[edgeID];
        for(int jd = 0; jd < dofs.size(); jd++)
        {
            int varID = id + nE_rho();
            if(map_dof_entire2subset.find(dofs[jd]) != map_dof_entire2subset.end()){
                int irow = map_dof_entire2subset[dofs[jd]];
                dF_dKu(irow, varID) += df_dKu[jd];
            }
        }
    }
}


void FrameTOSupport::computeComplianceHessian(const Eigen::VectorXd &rho,
                                              const Eigen::VectorXd &fix,
                                              Eigen::MatrixXd &hessian)
{
    Eigen::VectorXd F;
    SparseMatrixD K_stiff, K_fix, K;
    computeStiffnessMatrix(rho, K_stiff);
    computeFixtureStiffnessMatrix(fix, K_fix);
    K = K_fix + K_stiff;
    computeForce(rho, F);

    Eigen::SimplicialLLT<SparseMatrixD> llt(K);
    if(llt.info() == Eigen::Success)
    {
        Eigen::VectorXd u = llt.solve(F);
        Eigen::MatrixXd dF_dKu;
        compute_dF_dKu(u, rho, fix, dF_dKu);
        hessian = 2.0 * dF_dKu.transpose() * llt.solve(dF_dKu);
    }
}


void FrameTOSupport::computeHessian(const double *x, double sigma, const double *lambda, double *hess) {
    Eigen::VectorXd rho, fix;
    computeVars(x, rho, fix);

    Eigen::MatrixXd hessian;
    computeComplianceHessian(rho , fix, hessian);

    std::vector<Eigen::Vector2i> index;
    computeHessIndex(index);
    for (int id = 0; id < index.size(); id++)
    {
        hess[id] = hessian(index[id].x(), index[id].y()) * sigma;
    }
}

void FrameTOSupport::computeConstraintsGradient(const double *x, double *c, double *jac)
{

    c[0] = -std::min(numTargetBeam_ - (int) start_partIDs_.size(), nE_rho());
    for (int id = 0; id < nE_rho(); id++) {
        c[0] += x[id];
    }

    c[1] = -std::min(numFixture_, nE_fix());
    for (int id = 0; id < nE_fix(); id++) {
        c[1] += x[id + nE_rho()];
    }

    for (int id = 0; id < nE_rho(); id++)
    {
        int fixID = rho_to_fix_[id] + nE_rho();
        c[id + 2] = x[id] - x[fixID];
    }

    for (int id = 0; id < nX(); id++) {
        jac[id] = 1;
    }

    for (int id = 0; id < nE_dynamic(); id++)
    {
        jac[nX() + 2 * id] = 1;
        jac[nX() + 2 * id + 1] = -1;
    }
}

void FrameTOSupport::computeConstraintsBound(std::vector<double> &cmin, std::vector<double> &cmax) {
    cmin.push_back(-0.01);
    cmax.push_back(0.01);
    cmin.push_back(-0.01);
    cmax.push_back(0.01);

    for (int id = 0; id + 2 < nC(); id++)
    {
        cmin.push_back(0);
        cmax.push_back(KN_INFINITY);
    }
}

void FrameTOSupport::computeJacIndex(std::vector<Eigen::Vector2i> &index) {

    for (int id = 0; id < nE_rho(); id++)
    {
        index.push_back(Eigen::Vector2i(0, id));
    }

    for (int id = 0; id < nE_fix(); id++) {
        index.push_back(Eigen::Vector2i(1, id + nE_rho()));
    }

    for (int id = 0; id < nE_rho(); id++)
    {
        int fixID = rho_to_fix_[id] + nE_rho();
        index.push_back(Eigen::Vector2i(id + 2, id));
        index.push_back(Eigen::Vector2i(id + 2, fixID));
    }
}

void FrameTOSupport::computeHessIndex(std::vector<Eigen::Vector2i> &index) {
    for (int id = 0; id < nX(); id++) {
        for (int jd = id; jd < nX(); jd++) {
            index.push_back(Eigen::Vector2i(id, jd));
        }
    }
}

void FrameTOSupport::computeVariableBounds(std::vector<double> &xmin, std::vector<double> &xmax) {
    for (int id = 0; id < nE_rho(); id++)
    {
        xmin.push_back(0);
        xmax.push_back(1);
    }
    for(int id = 0; id < nE_fix(); id++)
    {
        int partID = end_partIDs_[id];
        if(std::find(fixed_partIDs_.begin(), fixed_partIDs_.end(), partID) != fixed_partIDs_.end()){
            xmin.push_back(1);
            xmax.push_back(1);
        }
        else{
            xmin.push_back(0);
            xmax.push_back(1);
        }
    }
}

void FrameTOSupport::getXTypes(std::vector<int> &x_types, bool MINLP) {
    for (int id = 0; id < nX(); id++) {
        if (MINLP)
            x_types.push_back(KN_VARTYPE_BINARY);
        else
            x_types.push_back(KN_VARTYPE_CONTINUOUS);
    }
}

std::vector<double> FrameTOSupport::evaluateAssembly(Eigen::VectorXd &rho, Eigen::VectorXd &fix)
{
    std::vector<int> subset_beams;
    for (int id = 0; id < assembly_->edges_.size(); id++) {
        int edgeID = id;
        subset_beams.push_back(edgeID);
    }

    updateBeams(rho, fix);

    Eigen::VectorXd displacement;
    (std::static_pointer_cast<frame::FrameAssemblyFixture>(assembly_))->solveElasticity(subset_beams, displacement);

    double compliance = assembly_->computeCompliance(displacement, subset_beams);
    double max_displacement = assembly_->computeMaxDisplacement(displacement);

    std::vector<Eigen::VectorXd> internal_forces;
    std::vector<Eigen::Vector2d> joint_stresses;
    double max_stress = assembly_->computeMaxStress(subset_beams, displacement, internal_forces, joint_stresses);

    return {compliance, max_displacement, max_stress};
}

void FrameTOSupport::updateBeams(Eigen::VectorXd &rho, Eigen::VectorXd &fix) {
    assembly_->computeBeams();
    assembly_->computeBeamElasticity();

    for (int id = 0; id < assembly_->edges_.size(); id++) {
        int edgeID = id;
        double r = rho(id);
        assembly_->beams_[edgeID]->beam_width_ *= r;
        assembly_->beams_[edgeID]->beam_fixture_ = fix(id);
        assembly_->beams_elasticity_[edgeID]->beam_width_ *= r;
        assembly_->beams_elasticity_[edgeID]->material_.E *= computeSIMP(r, young_module_ratio_);
        assembly_->beams_elasticity_[edgeID]->material_.rho *= computeSIMP(r, weight_ratio_);
        assembly_->beams_elasticity_[edgeID]->beam_fixture_ = fix(id);
    }
}

void FrameTOSupport::initializeX(Eigen::VectorXd &x) {
    x = Eigen::VectorXd::Zero(nX());
    for (int id = 0; id < nE_rho(); id++) {
        x(id) = (double)(numTargetBeam_ - start_partIDs_.size()) / nE_rho();
    }
    for (int id = 0; id < nE_fix(); id++)
    {
        int partID = end_partIDs_[id];
        if (std::find(fixed_partIDs_.begin(), fixed_partIDs_.end(), partID) != fixed_partIDs_.end()) {
            x(id + nE_rho()) = 1;
        } else {
            x(id + nE_rho()) = (double)(numFixture_ - (double)fixed_partIDs_.size()) / nE_fix();
        }
    }
}

void FrameTOSupport::printSolution(const double *x) {
    Eigen::VectorXd rho, fix;
    computeVars(x, rho, fix);
    std::cout << "node = {";
    for (int id = 0; id < nE(); id++) {
        if (rho(id) > 0.5) {
            std::cout << id << ", ";
        }
    }
    std::cout << "}\n";

    std::cout << "support = {";
    for (int id = 0; id < nE(); id++) {
        if (fix(id) > 0.5) {
            std::cout << id << ", ";
        }
    }
    std::cout << "}\n";
}
double FrameTOSupport::computeCompliance(const Eigen::VectorXd &rho, const Eigen::VectorXd &fix, double stiffness_penalty) {
    SparseMatrixD K_rho;
    computeStiffnessMatrix(rho, K_rho);

    Eigen::VectorXd g;
    computeForce(rho, g);

    SparseMatrixD K_fix;
    computeFixtureStiffnessMatrix(fix, K_fix);

    SparseMatrixD K = stiffness_penalty * K_fix + K_rho;

    Eigen::SimplicialLLT<SparseMatrixD> llt(K);
    if(llt.info() == Eigen::Success){
        Eigen::VectorXd u = llt.solve(g);
        return 0.5 * g.dot(u);
    }
    return std::numeric_limits<double>::max();
}


//bool FrameTOSupport::computeDisplacementDerivatives(const Eigen::VectorXd &rho,
//                                                    const Eigen::VectorXd &fix,
//                                                    const SparseMatrixD &K,
//                                                    Eigen::VectorXd &u,
//                                                    Eigen::MatrixXd &du)
//{
//    Eigen::MatrixXd du_rho(total_dofs, rho.size());
//
//    Eigen::MatrixXd du_fix(total_dofs, fix.size());
//
//    Eigen::VectorXd g;
//    computeForce(rho, g);
//
//
//    Eigen::SimplicialLLT<SparseMatrixD> llt(K);
//
//    if (llt.info() == Eigen::Success) {
//        u = llt.solve(g);
//
//        //du/drho
//        {
//            std::vector<SparseMatrixD> dK;
//            computeStiffnessMatrixDerivatives(rho, dK);
//
//            std::vector<Eigen::VectorXd> dF;
//            computeForceDerivatives(rho, dF);
//
//            for (int id = 0; id < rho.size(); id++) {
//                Eigen::VectorXd du_i;
//                Eigen::VectorXd dK_iu = dK[id] * u;
//
//                du_i = llt.solve(dF[id]) - llt.solve(dK_iu);
//                du_rho.col(id) = du_i;
//            }
//        }
//
//        //du/dfix
//        {
//            std::vector<SparseMatrixD> dK;
//            computeFixtureStiffnessMatrixDerivatives(rho, dK);
//
//            for (int id = 0; id < fix.size(); id++) {
//                Eigen::VectorXd du_i;
//                Eigen::VectorXd dK_iu = dK[id] * u;
//
//                du_i =  -llt.solve(dK_iu);
//                du_fix.col(id) = du_i;
//            }
//        }
//
//        du = Eigen::MatrixXd(u.size(), rho.size() + fix.size());
//        du.block(0, 0, u.size(), rho.size()) = du_rho;
//        du.block(0, rho.size(), u.size(), fix.size()) = du_fix;
//        return true;
//    }
//    else
//    {
//        return false;
//    }
//}


}