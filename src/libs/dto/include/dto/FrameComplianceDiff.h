//
// Created by 汪子琦 on 01.12.22.
//

#ifndef PAVILLION_JSON_FRAMECOMPLIANCEDIFF_H
#define PAVILLION_JSON_FRAMECOMPLIANCEDIFF_H

#include <cmath>
#include "frame/FrameAssembly.h"
#include "knitro.h"
#include <iostream>

namespace dto
{
class FrameComplianceDiff
{
public:

    std::shared_ptr<frame::FrameAssembly> assembly_;

    typedef Eigen::SparseMatrix<double> SparseMatrixD;

public:

    double young_module_ratio_ = 1E-6;

    double weight_ratio_ = 0.0;

public:

    int total_dofs;

    std::unordered_map<int, int> map_dof_entire2subset;

    std::unordered_map<int, int> map_dof_subset2entire;

    std::vector<std::vector<int>> edge_dof_local2global;

    std::vector<Eigen::MatrixXd> beamStiff;

    std::vector<Eigen::VectorXd> beamWeight;

public:

    std::vector<int> dynamic_partIDs_;

public:

    FrameComplianceDiff(){
        dynamic_partIDs_.clear();
    }

    FrameComplianceDiff(const frame::FrameAssembly &assembly)
    {
        assembly_ = std::make_shared<frame::FrameAssembly>(assembly);
        for(int id = 0; id < assembly_->beams_.size(); id++){
            dynamic_partIDs_.push_back(id);
        }
    }

public:

    double computeSIMP(double rho, double ratio);

    double computeSIMPGradient(double rho, double ratio);

    void setParameters(double stiff_ratio = 1E-6,
                       double weight_ratio = 0
    ){
        young_module_ratio_ = stiff_ratio;
        weight_ratio_ = weight_ratio;
    }

    void computeStiffnessMatrix(const Eigen::VectorXd &rho, SparseMatrixD &K);

    void computeForce(const Eigen::VectorXd &rho, Eigen::VectorXd &g);

    bool computeDisplacement(const Eigen::VectorXd &rho, Eigen::VectorXd &u);

    double computeCompliance(const Eigen::VectorXd &rho);

    double computeMaxDisplacement(const Eigen::VectorXd &rho);

    double computeComplianceDerivatives(const Eigen::VectorXd &rho, const Eigen::VectorXd &u, Eigen::VectorXd &gradient);

    void computeComplianceHessian(const Eigen::VectorXd &rho, Eigen::MatrixXd &hessian);

public:

    void setup();

    void computeDoFMaps();

    void computeEdgeLocal2GlobalDof();

    void computeBeamStiffnWeight();

    void assembleStiffMatrix(std::vector<Eigen::Triplet<double>> &K_tri, const Eigen::MatrixXd &k_G, std::vector<int> &dofs_local2global);

    void assemblyForce(Eigen::VectorXd &F, const Eigen::VectorXd &g, std::vector<int> &dofs_local2global);

    void compute_dF_dKu(const Eigen::VectorXd &u, const Eigen::VectorXd &rho, Eigen::MatrixXd &dF_dKu);

};
}

#endif  //PAVILLION_JSON_FRAMECOMPLIANCEDIFF_H
