//
// Created by 汪子琦 on 31.10.22.
//
#include "dto/FrameTOSequenceDynamicHoldings.h"

void dto::FrameTOSequenceDynamicHoldings::initializeX(Eigen::VectorXd& x)
{
    x = Eigen::VectorXd::Zero(nX());

    for (int id = 0; id < nS(); id++)
    {
        for (int jd = 0; jd < nE_rho(); jd++)
        {
            int varID = id * nE_rho() + jd;
            x[varID] = (sections[id].x() - (int)start_partIDs_.size()) / (double)nE_rho();
        }
    }

    for(int id = 0; id < nS(); id++)
    {
        for(int jd = 0; jd < nE_fix(); jd++)
        {
            int varID = id * nE_fix() + jd;
            x[nRho() + varID] = (numFixture_) / (double)(nE_fix());
        }
    }
}

double dto::FrameTOSequenceDynamicHoldings::evaluateResult(std::vector<Eigen::VectorXd>& rhos, std::vector<Eigen::VectorXd>& fixes) {

    double result = std::numeric_limits<double>::min();
    for (int id = 0; id < nS(); id++)
    {
        int num_bar = 0;
        for(int jd = 0; jd < rhos[id].size(); jd++){
            num_bar += rhos[id][jd];
        }
        double compliance = computeCompliance(rhos[id], fixes[id], 1E5);
        std::cout << "bar "  << num_bar  << ": " << compliance << std::endl;
        result += compliance;
    }
    std::cout << "compliance" << ": " << result << std::endl;

    return result;
}

void dto::FrameTOSequenceDynamicHoldings::computeRhoFixConstraintDerivatives(const double *x, double *c, Eigen::MatrixXd &grad)
{
    //budget constraints
    int iC = 0;
    for (int id = 0; id < nS(); id++)
    {
        c[id] = -(sections[id].x() - (int)start_partIDs_.size());
        for (int jd = 0; jd < nE_rho(); jd++)
        {
            c[iC] += x[id * nE_rho() + jd];
        }
        iC++;
    }

    //ascent constraints
    for (int id = 1; id < nS(); id++)
    {
        for (int jd = 0; jd < nE_rho(); jd++)
        {
            int iA = (id - 1) * nE_rho() + jd;
            int iB = id * nE_rho() + jd;
            c[iC] = x[iB] - x[iA];
            iC++;
        }
    }

    // fix budget constraints
    for(int id = 0; id < nS(); id++)
    {
        c[iC] = -numFixture_;
        for(int jd = 0; jd < nE_fix(); jd++)
        {
            int varID = nRho() + id * nE_fix() + jd;
            c[iC] += x[varID];
        }
        iC++;
    }

    // fix ascend
    for(int id = 0; id < nS(); id++)
    {
        for(int jd = 0; jd < nE_rho(); jd++)
        {
            int rhoID = id * nE_rho() + jd;
            int fixID = id * nE_fix() + rho_to_fix_[jd] + nRho();
            c[iC] = x[rhoID] - x[fixID];
            iC++;
        }
    }

    //gradient
    iC = 0;
    for (int id = 0; id < nS(); id++)
    {
        for (int jd = 0; jd < nE_rho(); jd++)
        {
            int varID = nE_rho() * id + jd;
            grad(iC, varID) = 1;
        }
        iC++;
    }

    for (int id = 1; id < nS(); id++)
    {
        for (int jd = 0; jd < nE_rho(); jd++)
        {
            int iA = (id - 1) * nE_rho() + jd;
            int iB = id * nE_rho() + jd;
            grad(iC, iB) = 1;
            grad(iC, iA) = -1;
            iC++;
        }
    }

    // fix budget gradient
    for(int id = 0; id < nS(); id++)
    {
        for(int jd = 0; jd < nE_fix(); jd++)
        {
            int varID = nRho() + id * nE_fix() + jd;
            grad(iC, varID) = 1;
        }
        iC++;
    }

    //fix ascend gradient
    for(int id = 0; id < nS(); id++)
    {
        for(int jd = 0; jd < nE_rho(); jd++)
        {
            int rhoID = id * nE_rho() + jd;
            int fixID = id * nE_fix() + rho_to_fix_[jd] + nRho();
            grad(iC, rhoID) = 1;
            grad(iC, fixID) = -1;
            iC++;
        }
    }
}


void dto::FrameTOSequenceDynamicHoldings::computeRhosFixes(const double* x, std::vector<Eigen::VectorXd>& rhos, std::vector<Eigen::VectorXd>& fixes) {
    rhos.clear();
    for (int id = 0; id < nS(); id++)
    {
        Eigen::VectorXd rho = rho_base_;
        for (int jd = 0; jd < nE_rho(); jd++)
        {
            int partID = dynamic_partIDs_[jd];
            rho(partID) = x[id * nE_rho() + jd];
        }
        rhos.push_back(rho);
    }

    fixes.clear();
    for (int id = 0; id < nS(); id++)
    {
        Eigen::VectorXd fix = Eigen::VectorXd::Zero(nE());
        for(int jd = 0; jd < nE_fix(); jd++)
        {
            int partID = end_partIDs_[jd];
            int varID = id * nE_fix() + jd;
            fix(partID) = x[varID + nRho()];
        }
        fixes.push_back(fix);
    }
}
void dto::FrameTOSequenceDynamicHoldings::computeObjectiveGradient(const double *x, double *obj, double *obj_gradient)
{
    std::vector<Eigen::VectorXd> rhos;
    std::vector<Eigen::VectorXd> fixes;
    computeRhosFixes(x, rhos, fixes);

    *obj = 0;
    for(int id = 0; id < nX(); id++){
        obj_gradient[id] = 0;
    }

    for(int id = 0; id < rhos.size(); id++)
    {
        Eigen::VectorXd gradient;
        Eigen::VectorXd u;
        FrameTOSupport::computeDisplacement(rhos[id], fixes[id], u);
        double compliance = FrameTOSupport::computeComplianceDerivatives(rhos[id], fixes[id], u, gradient);
        *obj += compliance;

        for(int jd = 0; jd < nE_rho(); jd++)
        {
            int varI = id * nE_rho() + jd;
            obj_gradient[varI] += gradient[jd];
        }

        for(int jd = 0; jd < nE_fix(); jd++)
        {
            int varI = nRho() + id * nE_fix() + jd;
            obj_gradient[varI] += gradient[nE_rho() + jd];
        }

        // add scaffoldfree penalty
        Eigen::VectorXd dcompliance;
        Eigen::VectorXd displacement;
        FrameComplianceDiff::computeDisplacement(rhos[id], displacement);
        compliance = FrameComplianceDiff::computeComplianceDerivatives(rhos[id], displacement, dcompliance);
        *obj += compliance * scaffoldfree_penalty;

        for(int jd = 0; jd < nE_rho(); jd++)
        {
            int varI = id * nE_rho() + jd;
            obj_gradient[varI] += gradient[jd] * scaffoldfree_penalty;
        }
    }
}

void dto::FrameTOSequenceDynamicHoldings::computeConstraintsGradient(const double *x, double *c, double *jac)
{
    std::vector<Eigen::VectorXd> rhos;
    std::vector<Eigen::VectorXd> fixes;
    computeRhosFixes(x, rhos, fixes);

    for(int id = 0; id < nC(); id++) c[id] = 0;
    Eigen::MatrixXd gradient(nC(), nX());
    gradient.setZero();

    computeRhoFixConstraintDerivatives(x, c, gradient);

    std::vector<Eigen::Vector2i> jac_index;
    computeJacIndex(jac_index);
    for (int id = 0; id < jac_index.size(); id++)
    {
        jac[id] = gradient(jac_index[id][0], jac_index[id][1]);
    }
}

void dto::FrameTOSequenceDynamicHoldings::computeConstraintsBound(std::vector<double> &cmin, std::vector<double> &cmax) {
    for (int id = 0; id < nC_budget(); id++)
    {
        cmin.push_back(0);
        cmax.push_back(sections[id].y() - sections[id].x() + 0.01);
    }

    for (int id = 0; id < nC_ascend(); id++)
    {
        cmin.push_back(0);
        cmax.push_back(KN_INFINITY);
    }

    for(int id = 0; id < nC_Fix_budget(); id++)
    {
        cmin.push_back(-0.01);
        cmax.push_back(0.01);
    }

    for (int id = 0; id < nC_Fix_ascend(); id++)
    {
        cmin.push_back(0);
        cmax.push_back(KN_INFINITY);
    }
}

void dto::FrameTOSequenceDynamicHoldings::computeJacIndex(std::vector<Eigen::Vector2i> &index) {

    for (int id = 0; id < nS(); id++)
    {
        for (int jd = 0; jd < nE_rho(); jd++)
        {
            int varID = nE_rho() * id + jd;
            index.push_back(Eigen::Vector2i(id, varID));
        }
    }

    int iC = nS();
    for (int id = 1; id < nS(); id++)
    {
        for (int jd = 0; jd < nE_rho(); jd++)
        {
            int iA = (id - 1) * nE_rho() + jd;
            int iB = id * nE_rho() + jd;
            index.push_back(Eigen::Vector2i(iC, iB));
            index.push_back(Eigen::Vector2i(iC, iA));
            iC++;
        }
    }

    //fix budget
    for(int id = 0; id < nS(); id++){
        for(int jd = 0; jd < nE_fix(); jd++)
        {
            int varID = nRho() + jd + id * nE_fix();
            index.push_back({iC, varID});
        }
        iC++;
    }

    //fix ascend
    for(int id = 0; id < nS(); id++)
    {
        for(int jd = 0; jd < nE_rho(); jd++)
        {
            int rhoID = id * nE_rho() + jd;
            int fixID = rho_to_fix_[jd] + id * nE_fix() + nRho();
            index.push_back({iC, rhoID});
            index.push_back({iC, fixID});
            iC++;
        }
    }
}

void dto::FrameTOSequenceDynamicHoldings::computeHessian(const double *x, double sigma, const double *lambda, double *hess) {
    std::vector<Eigen::VectorXd> rhos;
    std::vector<Eigen::VectorXd> fixes;
    computeRhosFixes(x, rhos, fixes);

    std::vector<Eigen::MatrixXd> hessians;
    std::vector<Eigen::MatrixXd> hessians_scaffold_free;
    for(int id = 0; id < rhos.size(); id++){
        Eigen::MatrixXd hessian;
        FrameTOSupport::computeComplianceHessian(rhos[id], fixes[id], hessian);
        hessians.push_back(hessian);

        FrameComplianceDiff::computeComplianceHessian(rhos[id], hessian);
        hessians_scaffold_free.push_back(hessian * scaffoldfree_penalty);
    }

    std::vector<Eigen::Vector2i> hess_index;
    computeHessIndex(hess_index);
    for (int id = 0; id < hess_index.size(); id++)
    {
        int iX = hess_index[id][0];
        int iY = hess_index[id][1];

        if(iX < nRho() && iY < nRho()){
            int index = iX / nE_rho();
            int ir = iX % nE_rho();
            int ic = iY % nE_rho();
            hess[id] += (hessians[index]).coeff(ir, ic);
            hess[id] += (hessians_scaffold_free[index]).coeff(ir, ic);
        }
        else if(iX >= nRho() && iY >= nRho())
        {
            int index = (iX - nRho()) / nE_fix();
            int ir = (iX - nRho()) % nE_fix() + nE_rho();
            int ic = (iY - nRho()) % nE_fix() + nE_rho();
            hess[id] += hessians[index].coeff(ir, ic);
        }
        else if(iX < nRho() && iY >= nRho()){
            int index = iX / nE_rho();
            int ir = iX % nE_rho();
            int ic = (iY - nRho()) % nE_fix() + nE_rho();
            hess[id] += hessians[index].coeff(ir, ic);
        }
        else{
            int index = iY / nE_rho();
            int ir = (iX - nRho()) % nE_fix() + nE_rho();
            int ic = iY % nE_rho();
            hess[id] += hessians[index].coeff(ir, ic);
        }
    }
}

void dto::FrameTOSequenceDynamicHoldings::computeHessIndex(std::vector<Eigen::Vector2i> &index) {
    for (int id = 0; id < nS(); id++)
    {
        for(int jd = 0; jd < nE_rho(); jd ++)
        {
            for(int kd = jd ; kd < nE_rho(); kd++)
            {
                int iX = id * nE_rho() + jd;
                int iY = id * nE_rho() + kd;
                index.push_back(Eigen::Vector2i(iX, iY));
            }
        }
    }

    for(int id = 0; id < nS(); id++)
    {
        for(int jd = 0; jd < nE_rho(); jd++)
        {
            int varRho = id * nE_rho() + jd;
            for(int kd = 0; kd < nE_fix(); kd++)
            {
                int varFix = nRho() + id * nE_fix() + kd;
                index.push_back({varRho, varFix});
            }
        }
    }

    for (int id = 0; id < nS(); id++)
    {
        for(int jd = 0; jd < nE_fix(); jd++)
        {
            for(int kd = jd; kd < nE_fix(); kd++)
            {
                int iX = id * nE_fix() + jd + nRho();
                int iY = id * nE_fix() + kd + nRho();
                index.push_back({iX, iY});
            }
        }
    }

}

void dto::FrameTOSequenceDynamicHoldings::computeVariableBounds(std::vector<double> &xmin, std::vector<double> &xmax) {

    for (int id = 0; id < nX(); id++) {
        xmin.push_back(0);
        xmax.push_back(1);
    }
}

void dto::FrameTOSequenceDynamicHoldings::getXTypes(std::vector<int> &x_types, bool MINLP) {
    if(MINLP) x_types.resize(nX(), KN_VARTYPE_BINARY);
    else x_types.resize(nX(), KN_VARTYPE_CONTINUOUS);
}

void dto::FrameTOSequenceDynamicHoldings::printSolution(const double *x)
{
    std::vector<Eigen::VectorXd> rhos;
    std::vector<Eigen::VectorXd> fixes;
    computeRhosFixes(x, rhos, fixes);
    evaluateResult(rhos, fixes);
    for(int id = 0; id < nS(); id++)
    {
        std::cout << "node" << std::to_string(id) << "= { ";
        for(int jd = 0; jd < nE(); jd++){
            if(rhos[id][jd] > 0.5){
                std::cout << jd << ", ";
            }
        }
        std::cout << "};\n";
    }

    for(int id = 0; id < nS(); id++)
    {
        std::cout << "fix" << std::to_string(id) << "= { ";
        for(int jd = 0; jd < nE(); jd++){
            if(fixes[id][jd] > 0.5){
                std::cout << jd << ", ";
            };
        }
        std::cout << "};\n";
    }
}

void dto::FrameTOSequenceDynamicHoldings::computePriorities(std::vector<int> &priorities) {
    priorities.clear();
    for(int id = 0; id < nS(); id++)
    {
        for(int jd = 0; jd < nE_rho(); jd++)
        {
            priorities.push_back(id + 1);
        }
    }

    for(int id = 0; id < nS(); id++)
    {
        for (int jd = 0; jd < nE_fix(); jd++)
        {
            priorities.push_back(nS() + id + 1);
        }
    }
}
