//
// Created by 汪子琦 on 03.05.22.
//

#ifndef ROBO_CRAFT_FRAMETO_H
#define ROBO_CRAFT_FRAMETO_H

#include <cmath>
#include "frame/FrameAssembly.h"
#include "knitro.h"
#include <iostream>
#include "search/PartGraphFrame.h"
#include "FrameComplianceDiff.h"
//#include <Eigen/CholmodSupport>

namespace dto
{

class FrameTO : public FrameComplianceDiff{
public:

    int numTargetBeam_;

    Eigen::VectorXd rho_base_;

    std::vector<int> start_partIDs_;

    std::vector<int> end_partIDs_;

    std::vector<int> rho_to_fix_;

public:

    FrameTO(): FrameComplianceDiff(){
        start_partIDs_ = {};
        end_partIDs_ = {};
    }

    FrameTO(const frame::FrameAssembly &assembly) : FrameComplianceDiff(assembly)
    {
        start_partIDs_ = {};
        for(int id = 0; id < assembly_->beams_.size(); id++){
            end_partIDs_.push_back(id);
        }
        setStartnEnd(start_partIDs_, end_partIDs_);
    }

public:

    int nE_dynamic(){return dynamic_partIDs_.size();}

    virtual void setStartnEnd(std::vector<int> start_partIDs, std::vector<int> end_partIDs);

    int nTarget(){return numTargetBeam_;}

    void setTarget(int targetBeam) {numTargetBeam_ = targetBeam;}

    int get_dynamic_id(int partID);

public:

    void computeRho(const double *x, Eigen::VectorXd &rho);

    virtual void initializeX(Eigen::VectorXd &x);

    virtual void getXTypes(std::vector<int> &x_types, bool MINLP = false);

public:

    virtual int nX();

    virtual int nC(){
        return 1;
    }

    virtual void computeObjectiveGradient(const double *x, double *obj, double *obj_grad);

    virtual void computeConstraintsGradient(const double *x, double *c, double *jac);

    virtual void computeConstraintsBound(std::vector<double> &cmin, std::vector<double> &cmax);

    virtual void computePriorities(std::vector<int> &priorities);

    virtual void computeHessian(const double *x, double sigma, const double *lambda, double *hess);

    virtual void computeJacIndex(std::vector<Eigen::Vector2i> &index);

    virtual void computeHessIndex(std::vector<Eigen::Vector2i> &index);

    virtual void computeVariableBounds(std::vector<double> &xmin, std::vector<double> &xmax);

public:

    void updateBeams(Eigen::VectorXd &rho);

    std::vector<double> evaluateAssembly(Eigen::VectorXd &rho);

    virtual void printSolution(const double *x);

};
}

#endif  //ROBO_CRAFT_FRAMETO_H
