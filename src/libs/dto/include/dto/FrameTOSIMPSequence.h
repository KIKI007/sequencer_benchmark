//
// Created by 汪子琦 on 03.05.22.
//

#ifndef ROBO_CRAFT_FrameTOSIMP_H
#define ROBO_CRAFT_FrameTOSIMP_H

#include <cmath>
#include "frame/FrameAssembly.h"
#include "frame/FrameAssemblyExternalForces.h"
#include "knitro.h"
#include <iostream>

namespace dto
{
class FrameTOSIMP
{

public:

    std::shared_ptr<frame::FrameAssembly> assembly_;

    typedef Eigen::SparseMatrix<double> SparseMatrixD;

protected:

    double young_module_ratio_ = 1E-6;

    double weight_ratio_ = 0.0;

    Eigen::VectorXd external_force_;

public:

    int simp_penlty_ = 1;

    bool budget_equal = true;

public:

    int total_dofs;

public:

    int numTargetBeam_;

    std::unordered_map<int, int> map_dof_entire2subset;

    std::unordered_map<int, int> map_dof_subset2entire;

    std::vector<std::vector<int>> edge_dof_local2global;

    std::vector<Eigen::MatrixXd> beamStiff;

    std::vector<Eigen::VectorXd> beamWeight;

public:

    FrameTOSIMP(){

    }

    FrameTOSIMP(const frame::FrameAssembly &assembly)
    {
        assembly_ = std::make_shared<frame::FrameAssembly>(assembly);
    }

    FrameTOSIMP(const frame::FrameAssemblyExternalForces &assembly)
    {
        assembly_ = std::make_shared<frame::FrameAssemblyExternalForces>(assembly);
        setExternalForce(assembly.external_force_);
    }

    FrameTOSIMP(const dto::FrameTOSIMP &frameTo);

public:

    int nTarget(){return numTargetBeam_;}

    void setExternalForce(Eigen::VectorXd force);

    void setTarget(int targetBeam) {numTargetBeam_ = targetBeam;}

    double computeSIMP(double rho, double ratio, int power);

    double computeSIMPGradient(double rho, double ratio, int power);

    double computeSIMPHessian(double rho, double ratio, int power);

    void setParameters(double disp_tol = 0.0,
                       double disp_obj_weight = 0,
                       double comp_obj_weight = 1,
                       double stiff_ratio = 1E-6,
                       double weight_ratio = 0
                       ){
        young_module_ratio_ = stiff_ratio;
        weight_ratio_ = weight_ratio;
    }

public:

    void computeStiffnessMatrix(const Eigen::VectorXd &rho, SparseMatrixD &K);

    void computeStiffnessMatrixDerivatives(const Eigen::VectorXd &rho, std::vector<SparseMatrixD> &dK);

    void computeForce(const Eigen::VectorXd &rho, Eigen::VectorXd &g);

    void computeForceDerivatives(const Eigen::VectorXd &rho, std::vector<Eigen::VectorXd> &dF);

    bool computeDisplacement(const Eigen::VectorXd &rho,
                             Eigen::VectorXd &u);

    bool computeDisplacementDerivatives(const Eigen::VectorXd &rho,
                                        Eigen::VectorXd &u,
                                        Eigen::MatrixXd &du,
                                        SparseMatrixD &K);


    double computeComplianceDerivatives(const Eigen::VectorXd &rho,
                                        const Eigen::VectorXd &u,
                                        Eigen::VectorXd &gradient);

    void computeComplianceHessian(const Eigen::VectorXd &rho, SparseMatrixD &hessian);

    void computeComplianceHessian(const Eigen::VectorXd &rho, Eigen::MatrixXd &hessian);


    void computeComplianceHessian(const Eigen::VectorXd &rho,
                                  const Eigen::VectorXd &u,
                                  const Eigen::MatrixXd &du,
                                  SparseMatrixD &K,
                                  Eigen::MatrixXd &hessian);

    void computeComplianceHessianVector(const Eigen::VectorXd &rho,
                                        const Eigen::VectorXd &vec,
                                        double *hessian);


public:

    void setup();

    void computeDoFMaps();

    void computeEdgeLocal2GlobalDof();

    void computeBeamStiffnWeight();

    void assembleStiffMatrix(std::vector<Eigen::Triplet<double>> &K_tri, const Eigen::MatrixXd &k_G, std::vector<int> &dofs_local2global);

    void assemblyForce(Eigen::VectorXd &F, const Eigen::VectorXd &g, std::vector<int> &dofs_local2global);

public:

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

    virtual void computeHessian(const double *x, double sigma, const double *lambda, double *hess);

    virtual void computeHessianVector(const double *x, const double * v, double sigma, const double *lambda, double *hess);

    virtual void computeJacIndex(std::vector<Eigen::Vector2i> &index);

    virtual void computeHessIndex(std::vector<Eigen::Vector2i> &index);

    virtual void computeVariableBounds(std::vector<double> &xmin, std::vector<double> &xmax);

public:

    void updateBeams(Eigen::VectorXd &rho);

    std::vector<double> evaluateAssembly(Eigen::VectorXd &rho);

    virtual void printSolution(const double *x);

};
}

#endif  //ROBO_CRAFT_FrameTOSIMP_H
