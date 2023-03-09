//
// Created by 汪子琦 on 15.09.22.
//

#include "dto/FrameTOSequenceStaticScaffolds.h"
namespace dto{
void FrameTOSequenceStaticScaffolds::initializeX(Eigen::VectorXd &x)
{
    x = Eigen::VectorXd::Zero(nX());

    for (int id = 0; id < nS(); id++)
    {
        for (int jd = 0; jd < nE_rho(); jd++)
        {
            x[id * nE_rho() + jd] = (sections[id].x() - (int)start_partIDs_.size()) / (double)nE_dynamic();
        }
    }

    for(int id = 0; id < nE_fix(); id++)
    {
        int partID = end_partIDs_[id];
        if (std::find(fixed_partIDs_.begin(), fixed_partIDs_.end(), partID) != fixed_partIDs_.end()) {
            x(id + nRho()) = 1;
        } else {
            x(id + nRho()) = (double)(numFixture_ - (double)fixed_partIDs_.size()) / nE_fix();
        }
    }

    x(nX() - 1) = 0;
}


double FrameTOSequenceStaticScaffolds::evaluateResult(std::vector<Eigen::VectorXd> &rhos, Eigen::VectorXd &fix) {
    double result = std::numeric_limits<double>::min();
//    assembly_->beams_elasticity_.clear();
//    assembly_->computeBeamElasticity();

    for (int id = 0; id < rhos.size(); id++)
    {
        double compliance = computeCompliance(rhos[id], fix, 1E5);
        result = std::max(result, compliance);
    }

    return result;
}

void FrameTOSequenceStaticScaffolds::computeRhos(const double *x, std::vector<Eigen::VectorXd> &rhos, Eigen::VectorXd &fix)
{
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

    fix = Eigen::VectorXd::Zero(nE());
    for(int id = 0; id < nE_fix(); id++)
    {
        int partID = end_partIDs_[id];
        fix(partID) = x[id + nRho()];
    }
}

void FrameTOSequenceStaticScaffolds::computeObjectiveGradient(const double *x, double *obj, double *obj_gradient){
    *obj = x[nX() - 1];
    for(int id = 0; id < nX(); id++)
    {
        if(id + 1 == nX()){
            obj_gradient[id] = 1;
        }
        else{
            obj_gradient[id] = 0;
        }
    }
}

void FrameTOSequenceStaticScaffolds::computeRhoFixConstraintDerivatives(const double *x, double *c, Eigen::MatrixXd &grad)
{
    //budget constraints
    for (int id = 0; id < nS(); id++)
    {
        c[id] = -(sections[id].x() - (int)start_partIDs_.size());
        for (int jd = 0; jd < nE_rho(); jd++)
        {
            c[id] += x[id * nE_rho() + jd];
        }
    }

    //ascent constraints
    int iC = nS();
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
    c[iC] = -numFixture_;
    for(int id = 0; id < nE_fix(); id++){
        int varID = nRho() + id;
        c[iC] += x[varID];
    }
    iC++;

    // fix ascend
    for(int id = 0; id < nE_rho(); id++){
        int rhoID = (nS() - 1) * nE_rho() + id;
        int fixID = rho_to_fix_[id] + nRho();
        c[iC] = x[rhoID] - x[fixID];
        iC++;
    }

    //gradient
    for (int id = 0; id < nS(); id++)
    {
        for (int jd = 0; jd < nE_rho(); jd++)
        {
            int varID = nE_rho() * id + jd;
            grad(id, varID) = 1;
        }
    }

    iC = nS();
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

    for(int id = 0; id < nE_fix(); id++) {
        int varID = nRho() + id;
        grad(iC, varID) = 1;
    }
    iC++;

    for(int id = 0; id < nE_rho(); id++) {
        int rhoID = (nS() - 1) * nE_rho() + id;
        int fixID = rho_to_fix_[id] + nRho();
        grad(iC, rhoID) = 1;
        grad(iC, fixID) = -1;
        iC++;
    }
}

void FrameTOSequenceStaticScaffolds::computeMuConstraintDerivatives(const std::vector<Eigen::VectorXd> &rhos,
                                                                    const Eigen::VectorXd &fix,
                                                                    double mu,
                                                                    double *c,
                                                                    Eigen::MatrixXd &grad)
{
    int iC = nC_budget() + nC_ascend() + nC_Fix_budget() + nC_Fix_ascend();
    for(int id = 0; id < nS(); id++)
    {
        Eigen::VectorXd gradient;
        Eigen::VectorXd u;
        FrameTOSupport::computeDisplacement(rhos[id], fix, u);
        double compliance = FrameTOSupport::computeComplianceDerivatives(rhos[id], fix, u, gradient);
        c[iC] = compliance - mu;
        grad.block(iC, id * nE_rho(), 1, nE_rho()) += gradient.transpose().segment(0, nE_rho());
        grad.block(iC, nRho(), 1, nE_fix()) += gradient.transpose().segment(nE_rho(), nE_fix());
        grad(iC, nX() - 1) = -1;
        iC++;
    }
}

void FrameTOSequenceStaticScaffolds::computeConstraintsGradient(const double *x, double *c, double *jac) {

    std::vector<Eigen::VectorXd> rhos;
    Eigen::VectorXd fix;
    computeRhos(x, rhos, fix);
    double mu = x[nX() - 1];

    for(int id = 0; id < nC(); id++) c[id] = 0;
    Eigen::MatrixXd gradient(nC(), nX());
    gradient.setZero();

    computeRhoFixConstraintDerivatives(x, c, gradient);
    computeMuConstraintDerivatives(rhos, fix, mu, c, gradient);

    std::vector<Eigen::Vector2i> jac_index;
    computeJacIndex(jac_index);
    for (int id = 0; id < jac_index.size(); id++)
    {
        jac[id] = gradient(jac_index[id][0], jac_index[id][1]);
    }
}
void FrameTOSequenceStaticScaffolds::computeConstraintsBound(std::vector<double> &cmin, std::vector<double> &cmax) {
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

    {
        cmin.push_back(-0.01);
        cmax.push_back(0.01);
    }

    for (int id = 0; id < nC_Fix_ascend(); id++)
    {
        cmin.push_back(0);
        cmax.push_back(KN_INFINITY);
    }

    for(int id = 0; id < nC_mu(); id++){
        cmin.push_back(-KN_INFINITY);
        cmax.push_back(0);
    }
}
void FrameTOSequenceStaticScaffolds::computeJacIndex(std::vector<Eigen::Vector2i> &index)
{
    for (int id = 0; id < nS(); id++)
    {
        for (int jd = 0; jd < nE_rho(); jd++)
        {
            int varID = nE_rho() * id + jd;
            index.push_back(Eigen::Vector2i(id, varID));
        }
    }

    int iC = nS();
    for (int id = 1; id < nS(); id++) {
        for (int jd = 0; jd < nE_rho(); jd++) {
            int iA = (id - 1) * nE_rho() + jd;
            int iB = id * nE_rho() + jd;
            index.push_back(Eigen::Vector2i(iC, iB));
            index.push_back(Eigen::Vector2i(iC, iA));
            iC++;
        }
    }

    for(int id = 0; id < nE_fix(); id++)
    {
        int varID = nRho() + id;
        index.push_back({iC, varID});
    }
    iC++;

    for(int id = 0; id < nE_rho(); id++) {
        int rhoID = (nS() - 1) * nE_rho() + id;
        int fixID = rho_to_fix_[id] + nRho();
        index.push_back({iC, rhoID});
        index.push_back({iC, fixID});
        iC++;
    }

    for(int id = 0; id < nS(); id++)
    {
        for(int jd = 0; jd < nE_rho(); jd++)
        {
            int iA = id * nE_rho() + jd;
            index.push_back(Eigen::Vector2i(iC, iA));
            index.push_back(Eigen::Vector2i(iC, nRho() + jd));
        }
        index.push_back(Eigen::Vector2i(iC, nX() - 1));
        iC++;
    }
}

void FrameTOSequenceStaticScaffolds::computeHessian(const double *x, double sigma, const double *lambda, double *hess) {
    std::vector<Eigen::VectorXd> rhos;
    Eigen::VectorXd fix;
    computeRhos(x, rhos, fix);

    Eigen::MatrixXd hessian(nX(), nX());
    hessian.setZero();

    for (int id = 0; id < nS(); id++)
    {
        Eigen::MatrixXd hessian_r;
        computeComplianceHessian(rhos[id], fix, hessian_r);

        int varRho = id * nE_rho();
        int varFix = nRho();
        int iC = nC_budget() + nC_ascend() + nC_Fix_ascend() + nC_Fix_budget() + id;

        hessian.block(varRho, varRho, nE_rho(), nE_rho())
            += lambda[iC] * hessian_r.block(0, 0, nE_rho(), nE_rho());

        hessian.block(varFix, varRho, nE_fix(), nE_rho())
            += lambda[iC] * hessian_r.block(nE_rho(), 0, nE_fix(), nE_rho());

        hessian.block(varRho, varFix, nE_rho(), nE_fix())
            += lambda[iC] * hessian_r.block(0, nE_rho(), nE_rho(), nE_fix());

        hessian.block(varFix, varFix, nE_fix(), nE_fix())
            += lambda[iC] * hessian_r.block(nE_rho(), nE_rho(), nE_fix(), nE_fix());
    }

    std::vector<Eigen::Vector2i> hess_index;
    computeHessIndex(hess_index);
    for (int id = 0; id < hess_index.size(); id++)
    {
        hess[id] = hessian(hess_index[id][0], hess_index[id][1]);
    }
}

void FrameTOSequenceStaticScaffolds::computeHessIndex(std::vector<Eigen::Vector2i> &index)
{
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
                int varFix = nRho() + kd;
                index.push_back({varRho, varFix});
            }
        }
    }

    for(int id = 0; id < nE_fix(); id++)
    {
        for(int jd = id; jd < nE_fix(); jd++)
        {
            index.push_back({nRho() + id, nRho() + jd});
        }
    }
}
void FrameTOSequenceStaticScaffolds::computeVariableBounds(std::vector<double> &xmin, std::vector<double> &xmax) {

    for (int id = 0; id < nRho(); id++) {
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

    {
        xmin.push_back(0);
        xmax.push_back(KN_INFINITY);
    }
}

void FrameTOSequenceStaticScaffolds::getXTypes(std::vector<int> &x_types, bool MINLP) {
    if(MINLP) x_types.resize(nX() - 1, KN_VARTYPE_BINARY);
    else x_types.resize(nX() - 1, KN_VARTYPE_CONTINUOUS);
    x_types.push_back(KN_VARTYPE_CONTINUOUS);
}

void FrameTOSequenceStaticScaffolds::printSolution(const double *x)
{
    std::vector<Eigen::VectorXd> rhos;
    Eigen::VectorXd fix;
    computeRhos(x, rhos, fix);
    std::cout << "Compliance = " << evaluateResult(rhos, fix) << std::endl;
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

    std::cout << "fix = {";
    for(int id = 0; id < nE(); id++){
        if(fix[id] > 0.5){
            std::cout << id << ", ";
        };
    }
    std::cout << "};\n";
}
}
