//
// Created by 汪子琦 on 15.09.22.
//

#ifndef ROBO_CRAFT_FRAMETOSEQUENCESTATICSCAFFOLDSUM_H
#define ROBO_CRAFT_FRAMETOSEQUENCESTATICSCAFFOLDSUM_H

#include "FrameTOSupport.h"
namespace dto{
class FrameTOSequenceStaticScaffoldsSum : public FrameTOSupport
{
public:

    Eigen::Vector2i section_;
    int num_step_;
    int num_arm_;

public:
    FrameTOSequenceStaticScaffoldsSum(const frame::FrameAssembly &assembly) : FrameTOSupport(assembly)
    {

    }

public:

    void initializeX(Eigen::VectorXd &x) override;

    int nS(){return num_step_;}

    int nRho() {return nE_rho() * nS();}

    int nX() override{return nRho() + nE_fix();}

    int nC_arm() {return nS() + 1;}

    int nC_ascend() {return nE_rho() * (nS() - 1);}

    int nC_Fix_budget() {return 1;}

    int nC() override{return nC_arm() + nC_ascend() + nC_Fix_budget();}

public:

    double evaluateResult(std::vector<Eigen::VectorXd> &rhos, Eigen::VectorXd &fix);

    void computeRhos(const double *x, std::vector<Eigen::VectorXd> &rhos, Eigen::VectorXd &fix);

    void computeObjectiveGradient(const double *x, double *obj, double *obj_gradient) override;

public:


    void computeConstraintsGradient(const double *x, double *c, double *jac) override;

    void computeConstraintsBound(std::vector<double> &cmin, std::vector<double> &cmax) override;

    void computeJacIndex(std::vector<Eigen::Vector2i> &index) override;

    void computeHessian(const double *x, double sigma, const double *lambda, double *hess) override;

    void computeHessIndex(std::vector<Eigen::Vector2i> &index) override;

    void computeVariableBounds(std::vector<double> &xmin, std::vector<double> &xmax) override;

    void computePriorities(std::vector<int> &priorities) override;

    void getXTypes(std::vector<int> &x_types, bool MINLP) override;

    void printSolution(const double *x) override;

};
}


#endif  //ROBO_CRAFT_FRAMETOSEQUENCESTATICSCAFFOLDSUM_H
