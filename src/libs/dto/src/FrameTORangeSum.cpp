//
// Created by 汪子琦 on 06.09.22.
//

#include "dto/FrameTORangeSum.h"

void dto::FrameTORangeSum::initializeX(Eigen::VectorXd &x)
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
}

void dto::FrameTORangeSum::setStartnEnd(std::vector<int> start_partIDs,
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

    variable_types_.clear();
    variable_types_.resize(nX(),KN_VARTYPE_BINARY);
}

double dto::FrameTORangeSum::evaluateResult(std::vector<Eigen::VectorXd> &rhos)
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

void dto::FrameTORangeSum::computeRhos(const double *x, std::vector<Eigen::VectorXd> &rhos)
{
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

void dto::FrameTORangeSum::computeObjectiveGradient(const double *x, double *obj, double *obj_gradient) {
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
        *obj += compliance;
        for(int jd = 0; jd < dcompliance.size(); jd++){
            obj_gradient[jd + nV] += dcompliance[jd];
        }
        nV += nE_dynamic();
    }
}

void dto::FrameTORangeSum::computeConstraint(const double *x, double *c)
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

void dto::FrameTORangeSum::computeConstraintsGradient(const double *x, Eigen::MatrixXd &grad)
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

void dto::FrameTORangeSum::computeConstraintsGradient(const double *x, double *c, double *jac)
{
    //constraints
    std::vector<Eigen::VectorXd> rhos;
    computeRhos(x, rhos);

    //initalization
    for(int id = 0; id < nC(); id++){
        c[id] = 0;
    }

    computeConstraint(x, c);

    Eigen::MatrixXd grad = Eigen::MatrixXd::Zero(nC(), nX());
    computeConstraintsGradient(x, grad);

    std::vector<Eigen::Vector2i> jac_index;
    computeJacIndex(jac_index);
    for (int id = 0; id < jac_index.size(); id++)
    {
        jac[id] = grad(jac_index[id][0], jac_index[id][1]);
    }
}

void dto::FrameTORangeSum::computeConstraintsBound(std::vector<double> &cmin, std::vector<double> &cmax) {

    double eps = 0.01;
    for (int id = 0; id < sections_.size(); id++)
    {
        cmin.push_back(sections_[id].x() - eps - start_partIDs_.size() + 1);
        cmax.push_back(sections_[id].x() + eps - start_partIDs_.size() + num_arm_);

        cmin.push_back(sections_[id].y() - eps - start_partIDs_.size() - num_arm_);
        cmax.push_back(sections_[id].y() + eps - start_partIDs_.size() - 1);

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
}

void dto::FrameTORangeSum::computeJacIndex(std::vector<Eigen::Vector2i> &index) {

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
}

void dto::FrameTORangeSum::computeHessian(const double *x, double sigma, const double *lambda, double *hess)
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

void dto::FrameTORangeSum::computeHessIndex(std::vector<Eigen::Vector2i> &index) {
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

void dto::FrameTORangeSum::computeVariableBounds(std::vector<double> &xmin, std::vector<double> &xmax) {

    for (int id = 0; id < nS(); id++)
    {
        for(int kd = 0; kd < nE_dynamic(); kd++)
        {
            xmin.push_back(0);
            xmax.push_back(1);
        }
    }
}

void dto::FrameTORangeSum::getXTypes(std::vector<int> &x_types, bool MINLP) {
    if(MINLP){
        x_types.resize(nX(), KN_VARTYPE_BINARY);
        //x_types = variable_types_;
    }
    else{
        x_types.resize(nX(), KN_VARTYPE_CONTINUOUS);
    }
}

void dto::FrameTORangeSum::printSolution(const double *x)
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
        std::cout << rhos[id].sum() << ", " << compliance  << std::endl;
    }
}

void dto::FrameTORangeSum::setSections(std::vector<int> sections, std::vector<int> numSteps) {
    sections_.clear();
    num_steps_.clear();

    for(int id = 0; id < sections.size(); id++)
    {
        sections_.push_back(Eigen::Vector2i(sections[id], sections[id]));
        num_steps_.push_back(numSteps[id]);
    }
}
void dto::FrameTORangeSum::computePriorities(std::vector<int> &priorities) {
    priorities.clear();
    for(int id = 0; id < nS(); id++)
    {
        for(int jd = 0; jd < nE_dynamic(); jd++)
        {
            int iP = std::min(id, nS() - id);
            //priorities.push_back(iP + 1);
//            priorities.push_back(id + 1);
        }
    }
}
