//
// Created by 汪子琦 on 06.09.22.
//

#include "dto/FrameTORange.h"

void dto::FrameTORange::initializeX(Eigen::VectorXd &x)
{
    x = Eigen::VectorXd::Zero(nX());

    int iX = 0;
    for(int id = 0; id < sections_.size(); id++)
    {
        for(int jd = 0; jd < num_steps_[id]; jd++)
        {
            double num_part = std::min((double) sections_[id].y(),
                                       (double) jd * num_arm_ + sections_[id].x())
                                       - (double) start_partIDs_.size();
            for(int kd = 0; kd < nE_dynamic(); kd++){
                x(iX) = num_part / (double) nE_dynamic();
                iX++;
            }
        }
    }
    x(nX() - 1) = 0;
}

void dto::FrameTORange::setStartnEnd(std::vector<int> start_partIDs,
                                     std::vector<int> end_partIDs)
{
    start_partIDs_ = start_partIDs;
    end_partIDs_ = end_partIDs;
    sections_.clear();

    sections_.push_back({start_partIDs.size(), end_partIDs.size()});

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

double dto::FrameTORange::evaluateResult(std::vector<Eigen::VectorXd> &rhos)
{
    double result = std::numeric_limits<double>::min();
    assembly_->beams_elasticity_.clear();
    assembly_->computeBeamElasticity();
    for (int id = 0; id < rhos.size(); id++)
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
        }
        result = std::max(result, deformation);
    }

    return result;
}

void dto::FrameTORange::computeRhos(const double *x, std::vector<Eigen::VectorXd> &rhos) {
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

void dto::FrameTORange::computeObjectiveGradient(const double *x, double *obj, double *obj_gradient) {
    *obj = x[nX() - 1];
    for(int id = 0; id < nX(); id++){
        if(id + 1 == nX()){
            obj_gradient[id] = 1;
        }
        else{
            obj_gradient[id] = 0;
        }
    }
}

void dto::FrameTORange::computeConstraint(const double *x, double *c)
{
    int numStep = 0;
    int iC = 0;
    for(int id = 0; id < sections_.size(); id++)
    {
        //begin & end
        int steps[2] = {numStep, numStep + num_steps_[id] - 1};
        for(int jd = 0; jd < 2; jd++)
        {
            for(int kd = 0; kd < nE_dynamic(); kd++)
            {
                int varI = steps[jd] * nE_dynamic() + kd;
                c[iC] += x[varI];
            }
            iC++;
        }

        //in between
        for(int jd = 1; jd < num_steps_[id]; jd++)
        {
            for(int kd = 0; kd < nE_dynamic(); kd++)
            {
                int iA = (numStep + jd) * nE_dynamic() + kd;
                int iB = (numStep + jd - 1) * nE_dynamic() + kd;
                c[iC] += x[iA] - x[iB];
            }
            iC++;
        }

        numStep += num_steps_[id];
    }

    for (int id = 1; id < nS(); id++)
    {
        for (int jd = 0; jd < nE_dynamic(); jd++)
        {
            int iA = (id - 1) * nE_dynamic() + jd;
            int iB = id * nE_dynamic() + jd;
            c[iC] = x[iB] - x[iA];
            iC++;
        }
    }
}

void dto::FrameTORange::computeConstraintsGradient(const double *x, Eigen::MatrixXd &grad)
{
    std::vector<Eigen::VectorXd> rhos;
    computeRhos(x, rhos);

    int numStep = 0;
    int iC = 0;
    for(int id = 0; id < sections_.size(); id++)
    {
        //begin & end
        int steps[2] = {numStep, numStep + num_steps_[id] - 1};
        for(int jd = 0; jd < 2; jd++)
        {
            for(int kd = 0; kd < nE_dynamic(); kd++)
            {
                int varID = steps[jd] * nE_dynamic() + kd;
                grad(iC, varID) += 1;
            }
            iC++;
        }

        //in between
        for(int jd = 1; jd < num_steps_[id]; jd++)
        {
            for(int kd = 0; kd < nE_dynamic(); kd++)
            {
                int varI = (numStep + jd) * nE_dynamic() + kd;
                int varJ = (numStep + jd - 1) * nE_dynamic() + kd;
                grad(iC, varI) = 1;
                grad(iC, varJ) = -1;
            }
            iC++;
        }
        numStep += num_steps_[id];
    }

    for (int id = 1; id < nS(); id++)
    {
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

void dto::FrameTORange::computeConstraintsGradient(const double *x, double *c, double *jac)
{
    //constraints
    std::vector<Eigen::VectorXd> rhos;
    computeRhos(x, rhos);

    for(int id = 0; id < nC(); id++){
        c[id] = 0;
    }

    computeConstraint(x, c);

    Eigen::MatrixXd grad(nC(), nX());
    grad.setZero();
    computeConstraintsGradient(x, grad);

    int iC = nC_arm() + nC_ascend();

    for(int id = 0; id < nS(); id++)
    {
        Eigen::VectorXd dcompliance;
        Eigen::VectorXd displacement;
        computeDisplacement(rhos[id], displacement);
        double compliance = computeComplianceDerivatives(rhos[id], displacement, dcompliance);
        c[iC] = compliance - x[nX() - 1];
        grad.block(iC, id * nE_dynamic(), 1, nE_dynamic()) = dcompliance.transpose();
        grad(iC, nX() - 1) = -1;
        iC++;
    }

    std::vector<Eigen::Vector2i> jac_index;
    computeJacIndex(jac_index);
    for (int id = 0; id < jac_index.size(); id++)
    {
        jac[id] = grad(jac_index[id][0], jac_index[id][1]);
    }
}

void dto::FrameTORange::computeConstraintsBound(std::vector<double> &cmin, std::vector<double> &cmax) {

    double eps = 0.01;
    for (int id = 0; id < sections_.size(); id++)
    {
        cmin.push_back(sections_[id].x() - eps - start_partIDs_.size() + 1);
        cmax.push_back(sections_[id].x() + eps - start_partIDs_.size() + num_arm_);

        cmin.push_back(sections_[id].y() - eps - start_partIDs_.size());
        cmax.push_back(sections_[id].y() + eps - start_partIDs_.size());
        //cmin.push_back(sections_[id].y() - eps - start_partIDs_.size() - num_arm_);
        //cmax.push_back(sections_[id].y() + eps - start_partIDs_.size() - 1);

        for(int jd = 1; jd < num_steps_[id]; jd++)
        {
            cmin.push_back(-eps);
            cmax.push_back(num_arm_ + eps);
        }
    }

    for (int id = 0; id < nC_ascend(); id++)
    {
        cmin.push_back(0);
        cmax.push_back(KN_INFINITY);
    }

    for(int id = nC_ascend() + nC_arm(); id < nC(); id++)
    {
        cmin.push_back(-KN_INFINITY);
        cmax.push_back(0);
    }
}

void dto::FrameTORange::computeJacIndex(std::vector<Eigen::Vector2i> &index) {

    int numStep = 0;
    int iC = 0;
    for(int id = 0; id < sections_.size(); id++)
    {
        //begin & end
        int steps[2] = {numStep, numStep + num_steps_[id] - 1};
        for(int jd = 0; jd < 2; jd++)
        {
            for(int kd = 0; kd < nE_dynamic(); kd++)
            {
                int varID = steps[jd] * nE_dynamic() + kd;
                index.push_back({iC, varID});
            }
            iC++;
        }

        //in between
        for(int jd = 1; jd < num_steps_[id]; jd++)
        {
            for(int kd = 0; kd < nE_dynamic(); kd++)
            {
                int varI = (numStep + jd) * nE_dynamic() + kd;
                int varJ = (numStep + jd - 1) * nE_dynamic() + kd;
                index.push_back({iC, varI});
                index.push_back({iC, varJ});
            }
            iC++;
        }
        numStep += num_steps_[id];
    }

    for (int id = 1; id < nS(); id++)
    {
        for (int jd = 0; jd < nE_dynamic(); jd++)
        {
            int iA = (id - 1) * nE_dynamic() + jd;
            int iB = id * nE_dynamic() + jd;
            index.push_back({iC, iA});
            index.push_back({iC, iB});
            iC++;
        }
    }

    for(int id = 0; id < nS(); id++)
    {
        for(int jd = 0; jd < nE_dynamic(); jd++)
        {
            int iA = id * nE_dynamic() + jd;
            index.push_back(Eigen::Vector2i(iC, iA));
        }
        index.push_back(Eigen::Vector2i(iC, nX() - 1));
        iC++;
    }

}

void dto::FrameTORange::computeHessian(const double *x, double sigma, const double *lambda, double *hess)
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
    for (int id = 0; id < hess_index.size(); id++)
    {
        int index = hess_index[id][0] / nE_dynamic();
        int ir = hess_index[id][0] % nE_dynamic();
        int ic = hess_index[id][1] % nE_dynamic();
        hess[id] = (hessians[index]).coeff(ir, ic) * lambda[nC_ascend() + nC_arm() +  index];
    }
}

void dto::FrameTORange::computeHessIndex(std::vector<Eigen::Vector2i> &index) {
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

void dto::FrameTORange::computeVariableBounds(std::vector<double> &xmin, std::vector<double> &xmax) {

    for (int id = 0; id < nS(); id++)
    {
        for(int kd = 0; kd < nE_dynamic(); kd++)
        {
            xmin.push_back(0);
            xmax.push_back(1);
        }
    }

    {
        xmin.push_back(0);
        xmax.push_back(KN_INFINITY);
    }
}

void dto::FrameTORange::getXTypes(std::vector<int> &x_types, bool MINLP) {
    if(MINLP) x_types.resize(nX() - 1, KN_VARTYPE_BINARY);
    else x_types.resize(nX() - 1, KN_VARTYPE_CONTINUOUS);
    x_types.push_back(KN_VARTYPE_CONTINUOUS);
}

void dto::FrameTORange::printSolution(const double *x)
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
}

void dto::FrameTORange::setSections(std::vector<int> sections, std::vector<int> numSteps) {
    for(int id = 0; id < sections.size(); id++)
    {
        sections_.push_back(Eigen::Vector2i(sections[id], sections[id]));
        num_steps_.push_back(numSteps[id]);
    }
}