//
// Created by 汪子琦 on 06.09.22.
//

#ifndef ROBO_CRAFT_FrameTORange_H
#define ROBO_CRAFT_FrameTORange_H
#include "FrameTO.h"
namespace dto
{
class FrameTORange : public FrameTO
{
public:

    std::vector<Eigen::Vector2i> sections_;
    std::vector<int> num_steps_;
    int num_arm_ = 1;

public:

    FrameTORange(const frame::FrameAssembly &assembly): FrameTO(assembly)
    {

    }

public:

    void setStartnEnd(std::vector<int> start_partIDs, std::vector<int> end_partIDs) override;

    void initializeX(Eigen::VectorXd &x) override;

    int nE(){return assembly_->beams_.size();}

    int nE_dynamic(){return dynamic_partIDs_.size();}

    int nS()
    {
        int step = 0;
        for(int id = 0; id < num_steps_.size(); id++){
            step += num_steps_[id];
        }
        return step;
    }

    int nX() override
    {
        return nE_dynamic() * nS() + 1;
    }

    int nC_ascend() {return (nS() - 1) * nE_dynamic();}

    int nC_arm() {return nS() + num_steps_.size();}

    int nC_mu() {return nS();}

    int nC() override{
        return nC_ascend() + nC_arm() + nC_mu();
    }

public:

    void setSections(std::vector<int> sections, std::vector<int> numSteps);

    double evaluateResult(std::vector<Eigen::VectorXd> &rhos);

    void computeRhos(const double *x, std::vector<Eigen::VectorXd> &rhos);

    void computeObjectiveGradient(const double *x, double *obj, double *obj_gradient) override;

    void computeConstraint(const double *x, double *c);

    void computeConstraintsGradient(const double *x, Eigen::MatrixXd &grad);

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

#endif  //ROBO_CRAFT_FrameTORange_H
