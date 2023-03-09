//
// Created by 汪子琦 on 15.09.22.
//

#include "dto/FrameTOSequenceStaticScaffoldsSum.h"
namespace dto{
void FrameTOSequenceStaticScaffoldsSum::initializeX(Eigen::VectorXd &x)
{
    x = Eigen::VectorXd::Zero(nX());

    for (int id = 0; id < nS(); id++)
    {
        int num_of_bars = std::min((double) section_.y(), (double) id * num_arm_ + section_.x()) - (double) start_partIDs_.size();
        for (int jd = 0; jd < nE_rho(); jd++)
        {
            x[id * nE_rho() + jd] = num_of_bars / (double) nE_rho();
        }
    }

    for(int id = 0; id < nE_fix(); id++)
    {
        int partID = end_partIDs_[id];
        x(id + nRho()) = (double)numFixture_ / nE_fix();
    }
}


double FrameTOSequenceStaticScaffoldsSum::evaluateResult(std::vector<Eigen::VectorXd> &rhos, Eigen::VectorXd &fix) {
    double result = std::numeric_limits<double>::min();
    for (int id = 0; id < rhos.size(); id++)
    {
        int num_bar = 0;
        for(int jd = 0; jd < rhos[id].size(); jd++){
            num_bar += rhos[id][jd];
        }
        double compliance = computeCompliance(rhos[id], fix, 1E5);
        std::cout << "bar "  << num_bar  << ": " << compliance << std::endl;
        result = result + compliance;
    }
    std::cout << "compliance" << ": " << result << std::endl;

    return result;
}

void FrameTOSequenceStaticScaffoldsSum::computeRhos(const double *x, std::vector<Eigen::VectorXd> &rhos, Eigen::VectorXd &fix)
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

void FrameTOSequenceStaticScaffoldsSum::computeObjectiveGradient(const double *x, double *obj, double *obj_gradient)
{
    std::vector<Eigen::VectorXd> rhos;
    Eigen::VectorXd fix;
    computeRhos(x, rhos, fix);

    *obj = 0;
    for(int id = 0; id < nX(); id++){
        obj_gradient[id] = 0;
    }

    for(int id = 0; id < rhos.size(); id++)
    {
        Eigen::VectorXd gradient;
        Eigen::VectorXd u;
        FrameTOSupport::computeDisplacement(rhos[id], fix, u);
        double compliance = FrameTOSupport::computeComplianceDerivatives(rhos[id], fix, u, gradient);
        *obj += compliance;

        for(int jd = 0; jd < nE_rho(); jd++)
        {
            int varI = id * nE_rho() + jd;
            obj_gradient[varI] += gradient[jd];
        }

        for(int jd = 0; jd < nE_fix(); jd++)
        {
            int varI = nRho() + jd;
            obj_gradient[varI] += gradient[nE_rho() + jd];
        }
    }
}


void FrameTOSequenceStaticScaffoldsSum::computeConstraintsGradient(const double *x, double *c, double *jac)
{
    //budget constraints
    int steps[2] = {0, num_step_ - 1};
    int iC = 0, iJ = 0;
    for(int jd = 0; jd < 2; jd++)
    {
        for(int kd = 0; kd < nE_dynamic(); kd++)
        {
            int varI = steps[jd] * nE_dynamic() + kd;
            c[iC] += x[varI];
            jac[iJ] = 1; iJ++;
        }
        iC++;
    }

    //in between
    for(int jd = 1; jd < num_step_; jd++)
    {
        for(int kd = 0; kd < nE_dynamic(); kd++)
        {
            int iA = jd * nE_dynamic() + kd;
            int iB = (jd - 1) * nE_dynamic() + kd;
            c[iC] += x[iA] - x[iB];
            jac[iJ] = 1; iJ++;
            jac[iJ] = -1; iJ++;
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
            jac[iJ] = 1; iJ++;
            jac[iJ] = -1; iJ++;
            iC++;
        }
    }

    // fix budget constraints
    c[iC] = -numFixture_;
    for(int id = 0; id < nE_fix(); id++){
        int varID = nRho() + id;
        c[iC] += x[varID];
        jac[iJ] = 1; iJ++;
    }
}

void FrameTOSequenceStaticScaffoldsSum::computeConstraintsBound(std::vector<double> &cmin, std::vector<double> &cmax) {

    double eps = 0.01;

    // start
    cmin.push_back(section_.x() - eps - start_partIDs_.size());
    cmax.push_back(section_.x() + num_arm_ + eps);

    // end
    cmin.push_back(section_.y() - eps - start_partIDs_.size() - num_arm_);
    cmax.push_back(section_.y() + eps - start_partIDs_.size() - 1);

    // middle
    for(int id = 1; id < nS(); id++)
    {
        cmin.push_back(0);
        cmax.push_back(num_arm_);
    }

    //ascend
    for (int id = 0; id < nC_ascend(); id++)
    {
        cmin.push_back(0);
        cmax.push_back(KN_INFINITY);
    }

    //fix
    {
        cmin.push_back(-0.01);
        cmax.push_back(0.01);
    }
}
void FrameTOSequenceStaticScaffoldsSum::computeJacIndex(std::vector<Eigen::Vector2i> &index)
{
    //start and end
    int steps[2] = {0, num_step_ - 1};
    int iC = 0;
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
    for(int jd = 1; jd < num_step_; jd++)
    {
        for(int kd = 0; kd < nE_dynamic(); kd++)
        {
            int varI = (jd) * nE_dynamic() + kd;
            int varJ = (jd - 1) * nE_dynamic() + kd;
            index.push_back({iC, varI});
            index.push_back({iC, varJ});
        }
        iC++;
    }

    //ascend
    for (int id = 1; id < nS(); id++) {
        for (int jd = 0; jd < nE_rho(); jd++) {
            int iA = (id - 1) * nE_rho() + jd;
            int iB = id * nE_rho() + jd;
            index.push_back(Eigen::Vector2i(iC, iB));
            index.push_back(Eigen::Vector2i(iC, iA));
            iC++;
        }
    }

    //fix
    for(int id = 0; id < nE_fix(); id++)
    {
        int varID = nRho() + id;
        index.push_back({iC, varID});
    }
}

void FrameTOSequenceStaticScaffoldsSum::computeHessian(const double *x, double sigma, const double *lambda, double *hess)
{
    std::vector<Eigen::VectorXd> rhos;
    Eigen::VectorXd fix;
    computeRhos(x, rhos, fix);

    std::vector<Eigen::MatrixXd> hessians;
    for(int id = 0; id < rhos.size(); id++){
        Eigen::MatrixXd hessian;
        FrameTOSupport::computeComplianceHessian(rhos[id], fix, hessian);
        hessians.push_back(hessian);
    }

    std::vector<Eigen::Vector2i> hess_index;
    computeHessIndex(hess_index);

    for(int id = 0; id < hess_index.size(); id++){
        hess[id] = 0;
    }

    for (int id = 0; id < hess_index.size(); id++)
    {
        if(hess_index[id][0] < nRho() && hess_index[id][1] < nRho()){
            int index = hess_index[id][0] / nE_dynamic();
            int ir = hess_index[id][0] % nE_dynamic();
            int ic = hess_index[id][1] % nE_dynamic();
            hess[id] += (hessians[index]).coeff(ir, ic);
        }
        else if(hess_index[id][0] >=  nRho() && hess_index[id][1] >= nRho())
        {
            int ir = hess_index[id][0] - nRho();
            int ic = hess_index[id][1] - nRho();
            for(int jd = 0; jd < hessians.size(); jd++){
                hess[id] += hessians[jd].coeff(ir + nE_rho(), ic + nE_rho());
            }
        }
        else if(hess_index[id][0] < nRho() && hess_index[id][1] >= nRho()){
            int index = hess_index[id][0] / nE_dynamic();
            int ir = hess_index[id][0] % nE_dynamic();
            int ic = hess_index[id][1] - nRho();
            hess[id] += hessians[index].coeff(ir, ic + nE_rho());
        }
        else{
            int index = hess_index[id][1] / nE_dynamic();
            int ir = hess_index[id][0] - nRho();
            int ic = hess_index[id][1] % nE_dynamic();
            hess[id] += hessians[index].coeff(ir + nE_rho(), ic);
        }
    }
}

void FrameTOSequenceStaticScaffoldsSum::computeHessIndex(std::vector<Eigen::Vector2i> &index)
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
void FrameTOSequenceStaticScaffoldsSum::computeVariableBounds(std::vector<double> &xmin, std::vector<double> &xmax) {

    for (int id = 0; id < nRho(); id++)
    {
        xmin.push_back(0);
        xmax.push_back(1);
    }

    for(int id = 0; id < nE_fix(); id++)
    {
        xmin.push_back(0);
        xmax.push_back(1);
    }
}

void FrameTOSequenceStaticScaffoldsSum::getXTypes(std::vector<int> &x_types, bool MINLP) {
    if(MINLP) x_types.resize(nX(), KN_VARTYPE_BINARY);
    else x_types.resize(nX(), KN_VARTYPE_CONTINUOUS);
}

void FrameTOSequenceStaticScaffoldsSum::printSolution(const double *x)
{
    std::vector<Eigen::VectorXd> rhos;
    Eigen::VectorXd fix;
    computeRhos(x, rhos, fix);
    evaluateResult(rhos, fix);
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
void FrameTOSequenceStaticScaffoldsSum::computePriorities(std::vector<int> &priorities) {
    priorities.clear();
    for(int id = 0; id < nS(); id++)
    {
        for(int jd = 0; jd < nE_rho(); jd++){
            priorities.push_back(id + 1);
        }
    }
    for(int id = 0; id < nE_fix(); id++)
    {
        priorities.push_back(nS() + 1);
    }
}

}
