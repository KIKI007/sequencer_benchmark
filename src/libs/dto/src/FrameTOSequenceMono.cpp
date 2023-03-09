//
// Created by 汪子琦 on 06.09.22.
//

#include "dto/FrameTOSequenceMono.h"

void dto::FrameTOSequenceMono::initializeX(Eigen::VectorXd &x)
{
    x = Eigen::VectorXd::Zero(nX());

    for (int id = 0; id < nS(); id++)
    {
        for (int jd = 0; jd < nE_dynamic(); jd++)
        {
            x[id * nE_dynamic() + jd] =  (sections_[id].x() - start_partIDs_.size()) / (double)nE_dynamic();
        }
    }
    int nV = nS() * nE_dynamic();
    for(int id = 0; id < nS(); id++){
        x[nV + id] = 0;
    }
}

void dto::FrameTOSequenceMono::setStartnEnd(std::vector<int> start_partIDs,
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

double dto::FrameTOSequenceMono::evaluateResult(std::vector<Eigen::VectorXd> &rhos)
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
            std::cout << deformation * 2 << std::endl;
        }
        result = std::max(result, deformation);
    }

    return result;
}

void dto::FrameTOSequenceMono::computeRhos(const double *x, std::vector<Eigen::VectorXd> &rhos) {
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

void dto::FrameTOSequenceMono::computeObjectiveGradient(const double *x, double *obj, double *obj_gradient) {
    *obj = 0;

    for(int id = 0; id < nX() - nS(); id++){
        obj_gradient[id] = 0;
    }

    int nV = nS() * nE_dynamic();
    for(int id = 0; id < nS(); id++){
        *obj += -x[nV + id];
        obj_gradient[nV  + id] = -1;
    }
}

void dto::FrameTOSequenceMono::computeConstraint(const double *x, double *c)
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

void dto::FrameTOSequenceMono::computeConstraintsGradient(const double *x, Eigen::MatrixXd &grad)
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

void dto::FrameTOSequenceMono::computeConstraintsGradient(const double *x, double *c, double *jac)
{
    std::vector<Eigen::VectorXd> rhos;
    computeRhos(x, rhos);

    computeConstraint(x, c);

    Eigen::MatrixXd grad;
    computeConstraintsGradient(x, grad);

    int iC = nC_budget() + nC_ascend();
    int nV = nE_dynamic() * nS();
    for(int id = 0; id < nS(); id++)
    {
        Eigen::VectorXd dcompliance;
        Eigen::VectorXd displacement;
        computeDisplacement(rhos[id], displacement);
        double compliance = computeComplianceDerivatives(rhos[id], displacement, dcompliance);
        c[iC] = compliance + x[nV + id];
        grad.block(iC, id * nE_dynamic(), 1, nE_dynamic()) = dcompliance.transpose();
        grad(iC, nV + id) = 1;
        iC++;
    }

    std::vector<Eigen::Vector2i> jac_index;
    computeJacIndex(jac_index);
    for (int id = 0; id < jac_index.size(); id++)
    {
        jac[id] = grad(jac_index[id][0], jac_index[id][1]);
    }
}

void dto::FrameTOSequenceMono::computeConstraintsBound(std::vector<double> &cmin, std::vector<double> &cmax) {
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

    for(int id = nC_budget() + nC_ascend(); id < nC(); id++){
        cmin.push_back(-KN_INFINITY);
        int is = id - nC_budget() - nC_ascend();
        cmax.push_back(sections_compliance_[is]);
    }
}

void dto::FrameTOSequenceMono::computeJacIndex(std::vector<Eigen::Vector2i> &index) {

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

    int nV = nS() * nE_dynamic();
    for(int id = 0; id < nS(); id++)
    {
        for(int jd = 0; jd < nE_dynamic(); jd++)
        {
            int iA = id * nE_dynamic() + jd;
            index.push_back(Eigen::Vector2i(iC, iA));
        }
        index.push_back(Eigen::Vector2i(iC, nV + id));
        iC++;
    }

}

void dto::FrameTOSequenceMono::computeHessian(const double *x, double sigma, const double *lambda, double *hess)
{
    std::vector<Eigen::VectorXd> rhos;
    computeRhos(x, rhos);

    std::vector<Eigen::MatrixXd> hessians;
    for (int id = 0; id < nS(); id++)
    {
        Eigen::MatrixXd hessian;
        computeComplianceHessian(rhos[id], hessian);
        hessian *= lambda[nC_budget() + nC_ascend() + id];
        hessians.push_back(hessian);
    }

    std::vector<Eigen::Vector2i> hess_index;
    computeHessIndex(hess_index);
    for (int id = 0; id < hess_index.size(); id++)
    {
        int index = hess_index[id][0] / nE_dynamic();
        int ir = hess_index[id][0] % nE_dynamic();
        int ic = hess_index[id][1] % nE_dynamic();
        hess[id] = hessians[index](ir, ic);
    }
}

void dto::FrameTOSequenceMono::computeHessIndex(std::vector<Eigen::Vector2i> &index) {
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

void dto::FrameTOSequenceMono::computeVariableBounds(std::vector<double> &xmin, std::vector<double> &xmax) {

    for (int id = 0; id < nS(); id++)
    {
        for(int jd = 0; jd < nE_dynamic(); jd++)
        {
            xmin.push_back(0);
            xmax.push_back(1);
        }
    }
    for(int id = 0; id < nS(); id++)
    {
        xmin.push_back(0);
        xmax.push_back(KN_INFINITY);
    }
}

void dto::FrameTOSequenceMono::getXTypes(std::vector<int> &x_types, bool MINLP) {
    if(MINLP) x_types.resize(nX() - nS(), KN_VARTYPE_BINARY);
    else x_types.resize(nX() - nS(), KN_VARTYPE_CONTINUOUS);
    for(int id = 0; id < nS(); id++) x_types.push_back(KN_VARTYPE_CONTINUOUS);
}

void dto::FrameTOSequenceMono::printSolution(const double *x)
{
    std::vector<Eigen::VectorXd> rhos;
    computeRhos(x, rhos);
    for(int id = 0; id < nS(); id++)
    {
        std::cout << "node" << std::to_string(id + 1) << "= { ";
        for(int jd = 0; jd < nE(); jd++)
        {
            if(rhos[id][jd] > 0.5){
                std::cout << jd << ", ";
            }
        }
        std::cout << "};\n";
    }
}

void dto::FrameTOSequenceMono::setSections(std::vector<int> sections, std::vector<double> sections_complaince) {
    sections_.clear();
    for(int id = 0; id < sections.size(); id++)
    {
        sections_.push_back(Eigen::Vector2i(sections[id], sections[id]));
    }
    sections_compliance_ = sections_complaince;
}
