//
// Created by 汪子琦 on 14.09.22.
//

#ifndef ROBO_CRAFT_FRAMETOFIXTURE_H
#define ROBO_CRAFT_FRAMETOFIXTURE_H

#include "FrameTO.h"
#include "frame/FrameAssemblyFixture.h"
namespace dto{
class FrameTOSupport : public FrameTO
{
public:

    int numFixture_ = 0;

    double fixture_stiffness_ = 1E3;

public:

    std::vector<int> fixed_partIDs_;

public:

    FrameTOSupport(const frame::FrameAssembly &assembly)
    {
        assembly_ = std::make_shared<frame::FrameAssemblyFixture>(assembly);
        start_partIDs_ = {};
        for(int id = 0; id < assembly_->beams_.size(); id++){
            end_partIDs_.push_back(id);
        }
        setStartnEnd(start_partIDs_, end_partIDs_);
    }

public:

    int nE(){
        return assembly_->beams_.size();
    }

    int nE_rho(){
        return nE_dynamic();
    }

    int nE_fix(){
        return end_partIDs_.size();
    }

    int nX() override{
        return nE_dynamic() + nE_fix();
    }

    virtual int nC() override
    {
        return 2 + nE_dynamic();
    }

    void setFixture(int numFix) {numFixture_ = numFix;}

    void computeFixtureStiffnessMatrix(const Eigen::VectorXd &fix, SparseMatrixD &K_fixture);

    void compute_dF_dKu(const Eigen::VectorXd &u, const Eigen::VectorXd &rho, const Eigen::VectorXd &fix, Eigen::MatrixXd &dF_dKu);

    bool computeDisplacement(const Eigen::VectorXd &rho, const Eigen::VectorXd &fix, Eigen::VectorXd &u);

    double computeCompliance(const Eigen::VectorXd &rho, const Eigen::VectorXd &fix, double stiffness_penalty = 1);

    double computeComplianceDerivatives(const Eigen::VectorXd &rho, const Eigen::VectorXd &fix, const Eigen::VectorXd &u, Eigen::VectorXd &gradient);

    void computeComplianceHessian(const Eigen::VectorXd &rho, const Eigen::VectorXd &fix, Eigen::MatrixXd &hessian);

    void computeVars(const double *x, Eigen::VectorXd &rho, Eigen::VectorXd &fix);

    void computeObjectiveGradient(const double *x, double *obj, double *obj_gradient) override;

    void computeConstraintsGradient(const double *x, double *c, double *jac) override;

    void computeConstraintsBound(std::vector<double> &cmin, std::vector<double> &cmax) override;

    void computeJacIndex(std::vector<Eigen::Vector2i> &index) override;

    void computeHessian(const double *x, double sigma, const double *lambda, double *hess) override;

    void computeHessIndex(std::vector<Eigen::Vector2i> &index) override;

    void computeVariableBounds(std::vector<double> &xmin, std::vector<double> &xmax) override;

    void getXTypes(std::vector<int> &x_types, bool MINLP) override;

    std::vector<double> evaluateAssembly(Eigen::VectorXd &rho, Eigen::VectorXd &fix);

    void updateBeams(Eigen::VectorXd &rho, Eigen::VectorXd &fix);

    void initializeX(Eigen::VectorXd &x) override;

    void printSolution(const double *x) override;

};

}

#endif  //ROBO_CRAFT_FRAMETOFIXTURE_H
