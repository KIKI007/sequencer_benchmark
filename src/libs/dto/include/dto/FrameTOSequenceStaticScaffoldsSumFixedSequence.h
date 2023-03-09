//
// Created by 汪子琦 on 15.09.22.
//

#ifndef ROBO_CRAFT_FRAMETOSEQUENCESTATICSCAFFOLDSUMFIXEDSSEQUENCE_H
#define ROBO_CRAFT_FRAMETOSEQUENCESTATICSCAFFOLDSUMFIXEDSSEQUENCE_H

#include "FrameTOSupport.h"

namespace dto{
class FrameTOSequenceStaticScaffoldsSumFixedSequence: public FrameTOSupport {
public:

    std::vector<std::vector<int>> seq_partIDs_;

public:
    FrameTOSequenceStaticScaffoldsSumFixedSequence(const frame::FrameAssembly &assembly) : FrameTOSupport(assembly){

    }

public:

    void initializeX(Eigen::VectorXd &x) override;

    int nE(){return assembly_->beams_.size();}

    int nS(){return seq_partIDs_.size();}

    int nX() override{return nE();}

    int nC() override{return 1;}

public:

    double evaluateResult(std::vector<Eigen::VectorXd> &rhos, Eigen::VectorXd &fix);

    void computeRhos(const double *x, std::vector<Eigen::VectorXd> &rhos, Eigen::VectorXd &fix);

    void computeObjectiveGradient(const double *x, double *obj, double *obj_gradient) override;

public:

    double computeComplianceDerivativesWrtFix(const Eigen::VectorXd &rho, const Eigen::VectorXd &fix, const Eigen::VectorXd &u, Eigen::VectorXd &gradient);

    void computeComplianceHessianWrtFix(const Eigen::VectorXd &rho, const Eigen::VectorXd &fix, Eigen::MatrixXd &hessian);

    void compute_dF_dKu_WrtFix(const Eigen::VectorXd &u, const Eigen::VectorXd &rho, const Eigen::VectorXd &fix, Eigen::MatrixXd &dF_dKu);

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


#endif  //ROBO_CRAFT_FRAMETOSEQUENCESTATICSCAFFOLDSUMFIXEDSSEQUENCE_H
