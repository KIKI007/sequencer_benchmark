//
// Created by 汪子琦 on 15.09.22.
//

#include "dto/FrameTOSequenceStaticScaffoldsSumFixedSequence.h"
namespace dto{
void FrameTOSequenceStaticScaffoldsSumFixedSequence::initializeX(Eigen::VectorXd &x)
{
    x = Eigen::VectorXd::Zero(nX());

    for(int id = 0; id < nE(); id++)
    {
       x(id) = (double )numFixture_ / nE();
    }
}

double FrameTOSequenceStaticScaffoldsSumFixedSequence::evaluateResult(std::vector<Eigen::VectorXd> &rhos, Eigen::VectorXd &fix)
{
    double result = std::numeric_limits<double>::min();
    for (int id = 0; id < rhos.size(); id++)
    {
        double compliance = computeCompliance(rhos[id], fix, 1E5);
        std::cout << "compliance " << id << " = " << compliance << std::endl;
        result = std::max(result, compliance);
    }

    return result;
}

void FrameTOSequenceStaticScaffoldsSumFixedSequence::computeRhos(const double *x, std::vector<Eigen::VectorXd> &rhos, Eigen::VectorXd &fix)
{
    rhos.clear();
    for(int id = 0; id < seq_partIDs_.size(); id++){
        Eigen::VectorXd rho(nE());
        rho.setZero();
        for(int jd = 0; jd < seq_partIDs_[id].size(); jd++)
        {
            int partID = seq_partIDs_[id][jd];
            rho(partID) = 1;
        }
        rhos.push_back(rho);
    }

    fix = Eigen::VectorXd::Zero(nE());
    for(int id = 0; id < nE(); id++)
    {
        fix(id) = x[id];
    }
}

double FrameTOSequenceStaticScaffoldsSumFixedSequence::computeComplianceDerivativesWrtFix(const Eigen::VectorXd &rho,
                                                                                        const Eigen::VectorXd &fix,
                                                                                        const Eigen::VectorXd &u,
                                                                                        Eigen::VectorXd &gradient) {
    Eigen::VectorXd g;
    computeForce(rho, g);

    double obj = g.dot(u);

    Eigen::VectorXd u_all = Eigen::VectorXd::Zero(assembly_->vertices_.size() * 6);
    for (int id = 0; id < u.rows(); id++) {
        int old_dof = map_dof_subset2entire[id];
        u_all[old_dof] = u[id];
    }

    gradient = Eigen::VectorXd(nX());
    gradient.setZero();

    auto get_u_edge = [&](int edgeID)
    {
        std::vector<int> dofs = assembly_->edgeDofIndices[edgeID];
        Eigen::VectorXd u_edge(dofs.size());
        for (int jd = 0; jd < dofs.size(); jd++) {
            u_edge(jd) = u_all[dofs[jd]];
        }
        return u_edge;
    };

    for(int id = 0; id < nE(); id++)
    {
        Eigen::VectorXd u_edge = get_u_edge(id);
        gradient(id) = -u_edge.dot(u_edge) * fixture_stiffness_;
    }

    return obj;
}

void FrameTOSequenceStaticScaffoldsSumFixedSequence::computeObjectiveGradient(const double *x, double *obj, double *obj_gradient){
    std::vector<Eigen::VectorXd> rhos;
    Eigen::VectorXd fix;
    computeRhos(x, rhos, fix);

    //initialization
    *obj = 0;
    for(int id = 0; id < nX(); id++){
        obj_gradient[id] = 0;
    }

    for(int id = 0; id < nS(); id++)
    {
        Eigen::VectorXd gradient;
        Eigen::VectorXd u;
        FrameTOSupport::computeDisplacement(rhos[id], fix, u);
        double compliance = computeComplianceDerivativesWrtFix(rhos[id], fix, u, gradient);

        *obj += compliance;
        for(int jd = 0; jd < gradient.size(); jd++){
            obj_gradient[jd] += gradient[jd];
        }
    }
}

void FrameTOSequenceStaticScaffoldsSumFixedSequence::computeConstraintsGradient(const double *x, double *c, double *jac) {

    std::vector<Eigen::VectorXd> rhos;
    Eigen::VectorXd fix;
    computeRhos(x, rhos, fix);

    c[0] = -numFixture_;
    for(int id = 0; id < nE(); id++){
        c[0] += fix(id);
    }
    for (int id = 0; id < nE(); id++)
    {
        jac[id] = 1;
    }
}

void FrameTOSequenceStaticScaffoldsSumFixedSequence::computeConstraintsBound(std::vector<double> &cmin, std::vector<double> &cmax) {
    cmin.push_back(-0.01);
    cmax.push_back(0.01);
}

void FrameTOSequenceStaticScaffoldsSumFixedSequence::computeJacIndex(std::vector<Eigen::Vector2i> &index)
{
    for(int id = 0; id < nE(); id++)
    {
        index.push_back({0, id});
    }
}

void FrameTOSequenceStaticScaffoldsSumFixedSequence::compute_dF_dKu_WrtFix(const Eigen::VectorXd &u,
                                                                           const Eigen::VectorXd &rho,
                                                                           const Eigen::VectorXd &fix,
                                                                           Eigen::MatrixXd &dF_dKu) {
    Eigen::VectorXd u_all = Eigen::VectorXd::Zero(assembly_->vertices_.size() * 6);
    for (int id = 0; id < u.rows(); id++)
    {
        int old_dof = map_dof_subset2entire[id];
        u_all[old_dof] = u[id];
    }

    dF_dKu = Eigen::MatrixXd(total_dofs, nX());
    dF_dKu.setZero();

    auto get_u_edge = [&](int edgeID)
    {
        std::vector<int> dofs = assembly_->edgeDofIndices[edgeID];
        Eigen::VectorXd u_edge(dofs.size());
        for (int jd = 0; jd < dofs.size(); jd++) {
            u_edge(jd) = u_all[dofs[jd]];
        }
        return u_edge;
    };

    for(int id = 0; id < nE(); id++)
    {
        Eigen::VectorXd u_edge = get_u_edge(id);
        Eigen::VectorXd df_dKu = -fixture_stiffness_ * u_edge;

        std::vector<int> dofs = assembly_->edgeDofIndices[id];
        for(int jd = 0; jd < dofs.size(); jd++)
        {
            int varID = id;
            if(map_dof_entire2subset.find(dofs[jd]) != map_dof_entire2subset.end()){
                int irow = map_dof_entire2subset[dofs[jd]];
                dF_dKu(irow, varID) += df_dKu[jd];
            }
        }
    }
}

void FrameTOSequenceStaticScaffoldsSumFixedSequence::computeComplianceHessianWrtFix(const Eigen::VectorXd &rho, const Eigen::VectorXd &fix, Eigen::MatrixXd &hessian) {
    Eigen::VectorXd F;
    SparseMatrixD K_stiff, K_fix, K;
    computeStiffnessMatrix(rho, K_stiff);
    computeFixtureStiffnessMatrix(fix, K_fix);
    K = K_fix + K_stiff;
    computeForce(rho, F);

    Eigen::SimplicialLLT<FrameComplianceDiff::SparseMatrixD> llt(K);
    if(llt.info() == Eigen::Success)
    {
        Eigen::VectorXd u = llt.solve(F);
        Eigen::MatrixXd dF_dKu;
        compute_dF_dKu_WrtFix(u, rho, fix, dF_dKu);
        hessian = 2.0 * dF_dKu.transpose() * llt.solve(dF_dKu);
    }
}


void FrameTOSequenceStaticScaffoldsSumFixedSequence::computeHessian(const double *x, double sigma, const double *lambda, double *hess) {
    std::vector<Eigen::VectorXd> rhos;
    Eigen::VectorXd fix;
    computeRhos(x, rhos, fix);

    Eigen::MatrixXd hessian(nE(), nE());
    hessian.setZero();
    for(int id = 0; id < nS(); id++){
        Eigen::VectorXd rho = rhos[id];
        Eigen::MatrixXd h;
        computeComplianceHessianWrtFix(rho, fix, h);
        hessian += h;
    }

    std::vector<Eigen::Vector2i> hess_index;
    computeHessIndex(hess_index);
    for (int id = 0; id < hess_index.size(); id++)
    {
        hess[id] = hessian(hess_index[id][0], hess_index[id][1]);
    }
}

void FrameTOSequenceStaticScaffoldsSumFixedSequence::computeHessIndex(std::vector<Eigen::Vector2i> &index)
{
    for(int id =0; id < nE(); id++)
    {
        for(int jd = id; jd < nE(); jd++)
        {
            index.push_back({id, jd});
        }
    }
}
void FrameTOSequenceStaticScaffoldsSumFixedSequence::computeVariableBounds(std::vector<double> &xmin, std::vector<double> &xmax)
{
    for(int id = 0; id < nE(); id++)
    {
        xmin.push_back(0);
        xmax.push_back(1);
    }
}

void FrameTOSequenceStaticScaffoldsSumFixedSequence::getXTypes(std::vector<int> &x_types, bool MINLP) {
    if(MINLP) x_types.resize(nX(), KN_VARTYPE_BINARY);
    else x_types.resize(nX(), KN_VARTYPE_CONTINUOUS);
}

void FrameTOSequenceStaticScaffoldsSumFixedSequence::printSolution(const double *x)
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
