//
// Created by 汪子琦 on 06.09.22.
//

#include "dto/FrameTOSequenceSum.h"

void dto::FrameTOSequenceSum::initializeX(Eigen::VectorXd &x)
{
    x = Eigen::VectorXd::Zero(nX());

    for (int id = 0; id < nS(); id++)
    {
        for (int jd = 0; jd < nE_dynamic(); jd++)
        {
            x[id * nE_dynamic() + jd] =  (sections_[id].x() - start_partIDs_.size()) / (double)nE_dynamic();
        }
    }
}

void dto::FrameTOSequenceSum::setStartnEnd(std::vector<int> start_partIDs,
                                        std::vector<int> end_partIDs)
{
    start_partIDs_ = start_partIDs;
    end_partIDs_ = end_partIDs;
    sections_.clear();

    for(int id = start_partIDs.size() + 1; id + 1 < end_partIDs.size(); id+= 1)
    {
        sections_.push_back({id, id});
    }

    dynamic_partIDs_.clear();
    rho_base_ = Eigen::VectorXd::Zero(nE());
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
}

double dto::FrameTOSequenceSum::evaluateResult(std::vector<Eigen::VectorXd> &rhos)
{
    double result = std::numeric_limits<double>::min();
    assembly_->beams_elasticity_.clear();
    assembly_->computeBeamElasticity();
    for (int id = 0; id + 1 < rhos.size(); id++)
    {
        Eigen::VectorXd rho = rhos[id];
        std::vector<int> subset_beams;

        for (int jd = 0; jd < rho.size(); jd++) {
            if (rho[jd] > 0.5) {
                subset_beams.push_back(jd);
            }
        }

        Eigen::VectorXd displacement;
        double deformation = 0;
        if (!subset_beams.empty()) {
            assembly_->solveElasticity(subset_beams, {}, displacement);
            deformation = assembly_->computeCompliance(displacement, subset_beams);
            std::cout << deformation << std::endl;
        }
        result = std::max(result, deformation);
    }

    return result;
}

void dto::FrameTOSequenceSum::computeRhos(const double *x, std::vector<Eigen::VectorXd> &rhos) {
    rhos.clear();
    for (int id = 0; id < nS(); id++)
    {
        Eigen::VectorXd rho = rho_base_;
        for (int jd = 0; jd < nE_dynamic(); jd++)
        {
            int partID = dynamic_partIDs_[jd];
            rho(partID) = x[id * nE_dynamic() + jd];
        }
        rhos.push_back(rho);
    }
}

void dto::FrameTOSequenceSum::computeObjectiveGradient(const double *x, double *obj, double *obj_gradient) {
    std::vector<Eigen::VectorXd> rhos;
    computeRhos(x, rhos);

    //initialization
    *obj = 0;
    for(int id = 0; id < nX(); id++){
        obj_gradient[id] = 0;
    }

    int nV = 0;
    for(int id = 0; id < nS(); id++)
    {
        Eigen::VectorXd dcompliance;
        Eigen::VectorXd displacement;
        computeDisplacement(rhos[id], displacement);
        double compliance = computeComplianceDerivatives(rhos[id], displacement, dcompliance);
        *obj += compliance ;
        for(int jd = 0; jd < dcompliance.size(); jd++){
            obj_gradient[jd + nV] += dcompliance[jd] ;
        }
        nV += nE_dynamic();
    }
}

void dto::FrameTOSequenceSum::computeConstraint(const double *x, double *c)
{
    int iC = 0;
    for(int id = 0; id < nC(); id++){
        c[id] = 0;
    }

    for (int id = 0; id < nC_budget(); id++)
    {
        c[iC] = (double)-sections_[id].x() + (double )start_partIDs_.size();
        for (int jd = 0; jd < nE_dynamic(); jd++)
        {
            int varI = id * nE_dynamic() + jd;
            c[iC] += x[varI];
        }
        iC++;
    }

    for (int id = 1; id < nS(); id++)
    {
        for (int jd = 0; jd < nE_dynamic(); jd++) {
            int iA = (id - 1) * nE_dynamic() + jd;
            int iB = id * nE_dynamic() + jd;
            c[iC] = x[iB] - x[iA];
            iC++;
        }
    }

}

void dto::FrameTOSequenceSum::computeConstraintsGradient(const double *x, Eigen::MatrixXd &grad)
{
    grad = Eigen::MatrixXd(nC(), nX());
    grad.setZero();
    for (int id = 0; id < nS(); id++)
    {
        for (int jd = 0; jd < nE_dynamic(); jd++)
        {
            int varI = id * nE_dynamic() + jd;
            grad(id, varI) = 1;
        }
    }

    int iC = nC_budget();
    for (int id = 1; id < nS(); id++) {
        for (int jd = 0; jd < nE_dynamic(); jd++)
        {
            int iA = (id - 1) * nE_dynamic() + jd;
            int iB = id * nE_dynamic() + jd;
            grad(iC, iB) = 1;
            grad(iC, iA) = -1;
            iC++;
        }
    }
}

void dto::FrameTOSequenceSum::computeConstraintsGradient(const double *x, double *c, double *jac)
{
    std::vector<Eigen::VectorXd> rhos;
    computeRhos(x, rhos);

    computeConstraint(x, c);

    Eigen::MatrixXd grad;
    computeConstraintsGradient(x, grad);

    std::vector<Eigen::Vector2i> jac_index;
    computeJacIndex(jac_index);
    for (int id = 0; id < jac_index.size(); id++)
    {
        jac[id] = grad(jac_index[id][0], jac_index[id][1]);
    }
}

void dto::FrameTOSequenceSum::computeConstraintsBound(std::vector<double> &cmin, std::vector<double> &cmax) {
    for (int id = 0; id < nC_budget(); id++)
    {
        cmin.push_back(0);
        cmax.push_back(sections_[id].y() - sections_[id].x() + 0.01);
    }

    for (int id = nC_budget(); id < nC_budget() + nC_ascend(); id++)
    {
        cmin.push_back(0);
        cmax.push_back(KN_INFINITY);
    }
}

void dto::FrameTOSequenceSum::computeJacIndex(std::vector<Eigen::Vector2i> &index) {

    for (int id = 0; id < nS(); id++)
    {
        for (int jd = 0; jd < nE_dynamic(); jd++)
        {
            int varI = nE_dynamic() * id + jd;
            index.push_back(Eigen::Vector2i(id, varI));
        }
    }

    int iC = nS();
    for (int id = 1; id < nS(); id++)
    {
        for (int jd = 0; jd < nE_dynamic(); jd++)
        {
            int iA = (id - 1) * nE_dynamic() + jd;
            int iB = id * nE_dynamic() + jd;
            index.push_back(Eigen::Vector2i(iC, iB));
            index.push_back(Eigen::Vector2i(iC, iA));
            iC++;
        }
    }
}

void dto::FrameTOSequenceSum::computeHessian(const double *x, double sigma, const double *lambda, double *hess)
{
    std::vector<Eigen::VectorXd> rhos;
    computeRhos(x, rhos);

    std::vector<Eigen::MatrixXd> hessians;
    for (int id = 0; id < nS(); id++)
    {
        Eigen::MatrixXd hessian;
        computeComplianceHessian(rhos[id], hessian);
        hessians.push_back(hessian);
    }

    std::vector<Eigen::Vector2i> hess_index;
    computeHessIndex(hess_index);

    for(int id = 0; id < hess_index.size(); id++){
        hess[id] = 0;
    }

    for (int id = 0; id < hess_index.size(); id++)
    {
        int index = hess_index[id][0] / nE_dynamic();
        int ir = hess_index[id][0] % nE_dynamic();
        int ic = hess_index[id][1] % nE_dynamic();
        hess[id] += (hessians[index]).coeff(ir, ic) * sigma;
    }
}

void dto::FrameTOSequenceSum::computeHessIndex(std::vector<Eigen::Vector2i> &index) {
    for (int id = 0; id < nS(); id++)
    {
        for(int jd = 0; jd < nE_dynamic(); jd ++)
        {
            for(int kd = jd ; kd < nE_dynamic(); kd++)
            {
                int iX = id * nE_dynamic() + jd;
                int iY = id * nE_dynamic() + kd;
                index.push_back(Eigen::Vector2i(iX, iY));
            }
        }
    }
}

void dto::FrameTOSequenceSum::computeVariableBounds(std::vector<double> &xmin, std::vector<double> &xmax) {

    for (int id = 0; id < nS(); id++)
    {
        for(int jd = 0; jd < nE_dynamic(); jd++)
        {
            xmin.push_back(0);
            xmax.push_back(1);
        }
    }
}

void dto::FrameTOSequenceSum::getXTypes(std::vector<int> &x_types, bool MINLP) {
    if(MINLP) x_types.resize(nX(), KN_VARTYPE_BINARY);
    else x_types.resize(nX(), KN_VARTYPE_CONTINUOUS);
}

void dto::FrameTOSequenceSum::printSolution(const double *x)
{
    std::vector<Eigen::VectorXd> rhos;
    computeRhos(x, rhos);
    for(int id = 0; id < nS(); id++)
    {
        std::cout << "node" << std::to_string(id + 1) << "= { ";
        for(int jd = 0; jd < rhos[id].size(); jd++){
            if(rhos[id][jd] > 0.5){
                std::cout << jd << ", ";
            }
        }
        std::cout << "};\n";
    }

    for (int id = 0; id < rhos.size(); id++)
    {
        double compliance = computeCompliance(rhos[id]);
        double displacement = computeMaxDisplacement(rhos[id]);
        std::cout << rhos[id].sum() << ", " << compliance << ", " << displacement << std::endl;
    }
}

void dto::FrameTOSequenceSum::setSections(std::vector<int> sections) {
    for(int id = 0; id < sections.size(); id++){
        sections_.push_back(Eigen::Vector2i(sections[id], sections[id]));
    }
}

void dto::FrameTOSequenceSum::computePriorities(std::vector<int> &priorities) {
    priorities.clear();

    std::vector<std::pair<int, double>> list;
    for(int id = 0; id < dynamic_partIDs_.size(); id++){
        int partID = dynamic_partIDs_[id];
        list.push_back({id, assembly_->beams_[partID]->computeCenterY()});
    }

    std::sort(list.begin(), list.end(), [=](auto a, auto b){
        return a.second >  b.second;
    });

    std::vector<int> bar_prior;
    bar_prior.resize(nE_dynamic());

    for(int id = 0; id < list.size(); id++)
    {
        bar_prior[list[id].first] = id + 1;
    }

    for(int id = 0; id < nS(); id++)
    {
        for(int jd = 0; jd < nE_dynamic(); jd++)
        {
//            priorities.push_back(bar_prior[jd]);
//            priorities.push_back(id + 1);
        }
    }
}
