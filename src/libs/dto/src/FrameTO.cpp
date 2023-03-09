//
// Created by 汪子琦 on 03.05.22.
//

#include "dto/FrameTO.h"

namespace dto {

void FrameTO::computeRho(const double *x, Eigen::VectorXd &rho) {
    rho = rho_base_;
    for(int id = 0; id < nE_dynamic(); id++){
        int edgeID = dynamic_partIDs_[id];
        rho(edgeID) = x[id];
    }
}

void dto::FrameTO::setStartnEnd(std::vector<int> start_partIDs,
                                std::vector<int> end_partIDs)
{
    start_partIDs_ = start_partIDs;
    end_partIDs_ = end_partIDs;

    std::sort(start_partIDs.begin(), start_partIDs.end());
    std::sort(end_partIDs.begin(), end_partIDs.end());

    dynamic_partIDs_.clear();
    rho_base_ = Eigen::VectorXd::Zero(assembly_->beams_.size());
    for(int id = 0; id < end_partIDs.size(); id++)
    {
        int partID = end_partIDs[id];
        if(std::find(start_partIDs.begin(), start_partIDs.end(), partID) == start_partIDs.end())
        {
            dynamic_partIDs_.push_back(partID);
        }
        else{
            rho_base_[partID] = 1;
        }
    }

    rho_to_fix_.clear();
    for(int id = 0; id < dynamic_partIDs_.size(); id++){
        int partID = dynamic_partIDs_[id];
        auto it = std::lower_bound(end_partIDs.begin(), end_partIDs.end(), partID);
        rho_to_fix_.push_back(it - end_partIDs.begin());
    }
}

int FrameTO::nX() {
    return nE_dynamic();
}

void FrameTO::getXTypes(std::vector<int> &x_types, bool MINLP) {
    if (MINLP)
        x_types.resize(nX(), KN_VARTYPE_BINARY);
    else
        x_types.resize(nX(), KN_VARTYPE_CONTINUOUS);
}

void FrameTO::initializeX(Eigen::VectorXd &x)
{
    x = Eigen::VectorXd::Zero(nX());

    for (int id = 0; id < x.size(); id++)
    {
        x(id) = (double)(numTargetBeam_ - start_partIDs_.size()) / x.size();
    }
}

void FrameTO::computeObjectiveGradient(const double *x, double *obj, double *obj_grad) {
    Eigen::VectorXd rho;
    computeRho(x, rho);

    Eigen::VectorXd u;
    Eigen::MatrixXd du;
    SparseMatrixD K;

    Eigen::VectorXd dobj_r(rho.size());
    dobj_r.setZero();

    computeDisplacement(rho, u);
    *obj = computeComplianceDerivatives(rho, u, dobj_r);

    for (int id = 0; id < dobj_r.size(); id++) {
        obj_grad[id] = dobj_r(id);
    }
}

void FrameTO::computeConstraintsGradient(const double *x, double *c, double *jac) {
    c[0] = -std::min((int)(numTargetBeam_ - start_partIDs_.size()), nX());
    for (int id = 0; id < nX(); id++) {
        c[0] += x[id];
    }

    for (int id = 0; id < nX(); id++) {
        jac[id] = 1;
    }
}

void FrameTO::computeHessian(const double *x, double sigma, const double *lambda, double *hess) {
    Eigen::VectorXd rho;
    computeRho(x, rho);

    Eigen::MatrixXd hessian;
    computeComplianceHessian(rho, hessian);

    std::vector<Eigen::Vector2i> index;
    computeHessIndex(index);
    for (int id = 0; id < index.size(); id++)
    {
        hess[id] = hessian(index[id].x(), index[id].y()) * sigma;
    }
}
void FrameTO::computeJacIndex(std::vector<Eigen::Vector2i> &index) {
    for (int id = 0; id < nX(); id++) {
        index.push_back(Eigen::Vector2i(0, id));
    }
}

void FrameTO::computeHessIndex(std::vector<Eigen::Vector2i> &index) {
    for (int id = 0; id < nX(); id++)
    {
        for (int jd = id; jd < nX(); jd++)
        {
            index.push_back(Eigen::Vector2i(id, jd));
        }
    }
}

void FrameTO::computeVariableBounds(std::vector<double> &xmin, std::vector<double> &xmax) {
    for (int id = 0; id < nX(); id++) {
        xmin.push_back(0);
        xmax.push_back(1);
    }
}
void FrameTO::computeConstraintsBound(std::vector<double> &cmin, std::vector<double> &cmax) {
    cmin.push_back(-0.01);
    cmax.push_back(0.01);
}

std::vector<double> FrameTO::evaluateAssembly(Eigen::VectorXd &rho)
{
    std::vector<int> subset_beams;
    for (int id = 0; id < assembly_->edges_.size(); id++) {
        int edgeID = id;
        subset_beams.push_back(edgeID);
    }

    updateBeams(rho);

    Eigen::VectorXd displacement;
    assembly_->solveElasticity(subset_beams, {}, displacement);

    double compliance = assembly_->computeCompliance(displacement, subset_beams);
    double max_displacement = assembly_->computeMaxDisplacement(displacement);

    std::vector<Eigen::VectorXd> internal_forces;
    std::vector<Eigen::Vector2d> joint_stresses;
    double max_stress = assembly_->computeMaxStress(subset_beams, displacement, internal_forces, joint_stresses);

    return {compliance, max_displacement, max_stress};
}

void FrameTO::updateBeams(Eigen::VectorXd &rho) {
    assembly_->computeBeams();
    assembly_->computeBeamElasticity();

    for (int id = 0; id < assembly_->edges_.size(); id++)
    {
        int edgeID = id;
        double r = rho(id);
        assembly_->beams_[edgeID]->beam_width_ *= r;
        assembly_->beams_elasticity_[edgeID]->beam_width_ *= r;
        assembly_->beams_elasticity_[edgeID]->material_.E *= computeSIMP(r, young_module_ratio_);
        assembly_->beams_elasticity_[edgeID]->material_.rho *= computeSIMP(r, weight_ratio_);
        assembly_->beams_elasticity_[edgeID]->material_.width = assembly_->beams_[edgeID]->beam_width_;
    }
}

void FrameTO::printSolution(const double *x) {
    std::cout << "node = {";
    Eigen::VectorXd rho;
    computeRho(x, rho);

    for (int id = 0; id < rho.size(); id++)
    {
        if (rho[id] > 0.5) {
            std::cout << id << ", ";
        }
    }
    std::cout << "};\n";
}

int FrameTO::get_dynamic_id(int partID)
{
    auto it = std::lower_bound(dynamic_partIDs_.begin(), dynamic_partIDs_.end(), partID);
    if(it == dynamic_partIDs_.end()){
        return -1;
    }
    else{
        return (it - dynamic_partIDs_.begin());
    }
}
void FrameTO::computePriorities(std::vector<int> &priorities)
{
    priorities.clear();
    std::vector<std::pair<int, double>> list;
    for(int id = 0; id < dynamic_partIDs_.size(); id++){
        int partID = dynamic_partIDs_[id];
        list.push_back({id, assembly_->beams_[partID]->computeCenterY()});
    }

    std::sort(list.begin(), list.end(), [=](auto a, auto b){
        return a.second < b.second;
    });

    std::vector<int> bar_prior;
    bar_prior.resize(nE_dynamic());

    for(int id = 0; id < list.size(); id++)
    {
        bar_prior[list[id].first] = id + 1;
    }

//    for(int jd = 0; jd < nX(); jd++)
//    {
//        priorities.push_back(bar_prior[jd]);
//    }
}

//
//void FrameTO::computeAdjacentBeams()
//{
//    int n = assembly_->beams_.size();
//    AdjGraph = Eigen::MatrixXd::Zero(n, n);
//    search::PartGraphFrame graph = search::PartGraphFrame(assembly_);
//
//    for(int id = 0; id < n; id++){
//        std::vector<unsigned > adj_nodes = graph.computeNodeNeighbour(id);
//        AdjGraph(id, id) = n;
//        for(int jd = 0; jd < adj_nodes.size(); jd++){
//            AdjGraph(id, adj_nodes[jd]) = -1;
//        }
//    }
//
//    AdjGraph = AdjGraph.transpose() * AdjGraph;
//}

//void FrameTO::computeForceDerivatives(const Eigen::VectorXd &rho, std::vector<Eigen::VectorXd> &dF) {
//    for (int id = 0; id < dynamic_partIDs_.size(); id++)
//    {
//        int edgeID = dynamic_partIDs_[id];
//        Eigen::VectorXd dF_i = Eigen::VectorXd(total_dofs);
//        dF_i.setZero();
//        Eigen::VectorXd g = beamWeight[edgeID];
//        Eigen::VectorXd dg = computeSIMPGradient(rho[edgeID], weight_ratio_) * g;
//        assemblyForce(dF_i, dg, edge_dof_local2global[edgeID]);
//        dF.push_back(dF_i);
//    }
//}

//void FrameTO::computeStiffnessMatrixDerivatives(const Eigen::VectorXd &rho, std::vector<SparseMatrixD> &dK) {
//
//    for (int id = 0; id < dynamic_partIDs_.size(); id++)
//    {
//        int edgeID = dynamic_partIDs_[id];
//        std::vector<Eigen::Triplet<double>> tripleLists;
//        Eigen::MatrixXd k = beamStiff[edgeID];
//        Eigen::MatrixXd dk = computeSIMPGradient(rho[edgeID], young_module_ratio_) * k;
//        assembleStiffMatrix(tripleLists, dk, edge_dof_local2global[edgeID]);
//        SparseMatrixD dK_stiff = SparseMatrixD(total_dofs, total_dofs);
//        dK_stiff.setFromTriplets(tripleLists.begin(), tripleLists.end());
//        dK.push_back(dK_stiff);
//    }
//}

//double FrameTO::computeMaxDisplacementDerivatives(const Eigen::VectorXd &rho,
//                                                  const Eigen::VectorXd &u,
//                                                  const Eigen::MatrixXd &du,
//                                                  Eigen::VectorXd &gradient) {
//    //obj
//    double obj = 0;
//    {
//        for (int id = 0; id < u.rows(); id++) {
//            if (u(id) < -displacement_tolerance_)
//                obj += (u(id) + displacement_tolerance_) * (u(id) + displacement_tolerance_);
//            else if (u(id) > displacement_tolerance_)
//                obj += (u(id) - displacement_tolerance_) * (u(id) - displacement_tolerance_);
//        }
//    }
//
//    gradient = Eigen::VectorXd(rho.size());
//    gradient.setZero();
//    //gradient
//    {
//        for (int id = 0; id < u.rows(); id++) {
//            if (u(id) < -displacement_tolerance_) {
//                gradient += 2 * du.row(id).transpose() * (u(id) + displacement_tolerance_);
//            } else if (u(id) > displacement_tolerance_) {
//                gradient += 2 * du.row(id).transpose() * (u(id) - displacement_tolerance_);
//            }
//        }
//    }
//    return obj;
//}
//
//
//void FrameTO::computeMaxDisplacementHessian(const Eigen::VectorXd &u,
//                                            const Eigen::MatrixXd &du,
//                                            Eigen::MatrixXd &hessian)
//{
//    Eigen::VectorXd gradient = Eigen::VectorXd(du.cols());
//    gradient.setZero();
//    //gradient
//    {
//        for (int id = 0; id < u.rows(); id++) {
//            if (u(id) < -displacement_tolerance_) {
//                gradient += 2 * du.row(id).transpose() * (u(id) + displacement_tolerance_);
//            } else if (u(id) > displacement_tolerance_) {
//                gradient += 2 * du.row(id).transpose() * (u(id) - displacement_tolerance_);
//            }
//        }
//    }
//
//    hessian = gradient * gradient.transpose();
//}


}