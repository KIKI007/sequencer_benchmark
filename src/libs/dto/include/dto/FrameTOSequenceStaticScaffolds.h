//
// Created by 汪子琦 on 15.09.22.
//

#ifndef ROBO_CRAFT_FRAMETOSEQUENCESTATICSCAFFOLD_H
#define ROBO_CRAFT_FRAMETOSEQUENCESTATICSCAFFOLD_H

#include "FrameTOSupport.h"
namespace dto{
class FrameTOSequenceStaticScaffolds : public FrameTOSupport {
public:

    std::vector<Eigen::Vector2i> sections;

public:
    FrameTOSequenceStaticScaffolds(const frame::FrameAssembly &assembly) : FrameTOSupport(assembly){

    }

public:

    void initializeX(Eigen::VectorXd &x) override;

    int nE(){return assembly_->beams_.size();}

    int nS(){return sections.size();}

    int nRho() {return nE_rho() * nS();}

    int nX() override{return nRho() + nE_fix() + 1;}

    int nC_budget() {return nS();}

    int nC_ascend() {return nE_rho() * (nS() - 1);}

    int nC_Fix_budget() {return 1;}

    int nC_Fix_ascend(){return nE_rho();}

    int nC_mu() {return nS();}

    int nC() override{return nC_budget() + nC_ascend() + nC_Fix_budget() + nC_Fix_ascend() + nC_mu();}

public:

    double evaluateResult(std::vector<Eigen::VectorXd> &rhos, Eigen::VectorXd &fix);

    void computeRhos(const double *x, std::vector<Eigen::VectorXd> &rhos, Eigen::VectorXd &fix);

    void computeObjectiveGradient(const double *x, double *obj, double *obj_gradient) override;

public:

    void computeRhoFixConstraintDerivatives(const double *x, double *c, Eigen::MatrixXd &grad);

    void computeMuConstraintDerivatives(const std::vector<Eigen::VectorXd> &rhos,
                                        const Eigen::VectorXd &fix,
                                        double mu,
                                        double *c,
                                        Eigen::MatrixXd &grad);

public:


    void computeConstraintsGradient(const double *x, double *c, double *jac) override;

    void computeConstraintsBound(std::vector<double> &cmin, std::vector<double> &cmax) override;

    void computeJacIndex(std::vector<Eigen::Vector2i> &index) override;

    void computeHessian(const double *x, double sigma, const double *lambda, double *hess) override;

    void computeHessIndex(std::vector<Eigen::Vector2i> &index) override;

    void computeVariableBounds(std::vector<double> &xmin, std::vector<double> &xmax) override;

    void getXTypes(std::vector<int> &x_types, bool MINLP) override;

    void printSolution(const double *x) override;

};
}


#endif  //ROBO_CRAFT_FRAMETOSEQUENCESTATICSCAFFOLD_H
