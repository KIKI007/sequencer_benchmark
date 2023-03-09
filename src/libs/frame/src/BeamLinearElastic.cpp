//
// Created by 汪子琦 on 03.03.22.
//

#include "frame/BeamLinearElastic.h"
#include <iostream>
namespace frame
{
Eigen::Matrix2d BeamLinearElastic::torsional_stiffness_matrix(double L, double J, double G) {
    return axial_stiffness_matrix(L, J, G);
}

Eigen::Matrix2d BeamLinearElastic::axial_stiffness_matrix(double L, double A, double E) {
    Eigen::MatrixXd k = Eigen::Matrix2d::Ones(2, 2);
    k(0, 1) = -1;
    k(1, 0) = -1;
    double d_inv = (E * A) / L;
    k *= d_inv;
    return k;
}

Eigen::Matrix4d BeamLinearElastic::bending_stiffness_matrix(double L, double E, double Iz, int axis)
{
    Eigen::Matrix4d k = Eigen::Matrix4d::Zero(4, 4);
    int sign = axis == 2? 1 : -1;
    double LL = L * L;

    k.row(0) = Eigen::RowVector4d(12.0 / LL, sign * 6 / L, -12.0 / LL, sign * 6 / L);
    k.row(1) = Eigen::RowVector4d(sign * 6 / L, 4, sign * (-6 / L), 2 );
    k.row(2) = -k.row(0);
    k.row(3) = Eigen::RowVector4d(sign * 6 / L, 2, sign * (-6 / L), 4);

    k *= E*Iz/L;
    return k;
}

Eigen::MatrixXd BeamLinearElastic::create_local_stiffness_matrix(bool reduce_bending) {

    double L = beam_length_;
    double Ax = material_.Ax;
    double Jxx = material_.Jxx;
    double Iyy = material_.Iyy;
    double Izz = material_.Izz;
    double E = material_.E;
    double mu = material_.mu;

    double G = E/(2*(1+mu));
    Eigen::Matrix2d axial_x_k = axial_stiffness_matrix(L, Ax, E);
    Eigen::Matrix2d tor_x_k = torsional_stiffness_matrix(L, Jxx, G);
    Eigen::Matrix4d bend_z_k = bending_stiffness_matrix(L, E, Izz, 2);
    Eigen::Matrix4d bend_y_k = bending_stiffness_matrix(L, E, Iyy, 1);

    Eigen::MatrixXd k = Eigen::MatrixXd::Zero(12, 12);

    //K[np.ix_([0,6], [0,6])] += axial_x_k
    //K[np.ix_([3,9], [3,9])] += tor_x_k
    //K[np.ix_([1,5,7,11], [1,5,7,11])] += bend_z_k
    //K[np.ix_([2,4,8,10], [2,4,8,10])] += bend_y_k
    addToMatrix({0, 6}, {0, 6}, k, axial_x_k);

    if(frame_node_){
        addToMatrix({3, 9}, {3, 9}, k, tor_x_k);
        addToMatrix({1, 5, 7, 11}, {1, 5, 7, 11}, k, bend_z_k);
        addToMatrix({2, 4, 8, 10}, {2, 4, 8, 10}, k, bend_y_k);
    }
    return k;
}

void BeamLinearElastic::addToMatrix(std::vector<int> indexI, std::vector<int> indexJ, Eigen::MatrixXd &A, const Eigen::MatrixXd B) {
    for(int id = 0; id < indexI.size(); id++){
        for(int jd = 0; jd < indexJ.size(); jd++){
            int II = indexI[id];
            int JJ = indexJ[jd];
            A(II, JJ) += B(id, jd);
        }
    }
}

void BeamLinearElastic::addToMatrix(std::vector<int> indexI, std::vector<int> indexJ, std::vector<Eigen::Triplet<double>> &tri, const Eigen::MatrixXd B) {
    for(int id = 0; id < indexI.size(); id++){
        for(int jd = 0; jd < indexJ.size(); jd++){
            int II = indexI[id];
            int JJ = indexJ[jd];
            tri.push_back(Eigen::Triplet<double>(II, JJ, B(id, jd)));
        }
    }
}

Eigen::MatrixXd BeamLinearElastic::turn_diagblock(Eigen::MatrixXd R3){

    Eigen::MatrixXd R_LG = Eigen::MatrixXd::Zero(12, 12);
    for(int id = 0; id < 4; id++){
        R_LG.block(id * 3, id * 3, 3, 3) = R3;
    }
    return R_LG;
}

Eigen::MatrixXd BeamLinearElastic::create_global_stiffness_matrix(bool reduce_bending) {

    Eigen::MatrixXd k = create_local_stiffness_matrix(reduce_bending);

    Eigen::MatrixXd R = create_global_transformation_matrix();

    Eigen::MatrixXd R_LG = turn_diagblock(R);

    Eigen::MatrixXd k_G = R_LG.transpose() * (k) * (R_LG);

    return k_G;
}

Eigen::MatrixXd BeamLinearElastic::create_global_transformation_matrix() {
    Eigen::Vector3d end_vert_u = frame_.origin;
    Eigen::Vector3d end_vert_v = frame_.origin + frame_.zaxis * beam_length_;
    double L = beam_length_;

    // by convention, the new x axis is along the element's direction
    // directional cosine of the new x axis in the global world frame
    double c_x = (end_vert_v[0] - end_vert_u[0])/L;
    double c_y = (end_vert_v[1] - end_vert_u[1])/L;
    double c_z = (end_vert_v[2] - end_vert_u[2])/L;
    Eigen::Matrix3d R = Eigen::Matrix3d::Zero();

    if(abs(abs(c_z) - 1.0) < 1E-8){
        R(0, 2) = c_z;
        R(1, 1) = 1;
        R(2, 0) = -c_z;
    }
    else{
        //local x_axis = element's vector
        Eigen::Vector3d new_x(c_x, c_y, c_z);
        //local y axis = cross product with global z axis
        Eigen::Vector3d new_y = -new_x.cross(Eigen::Vector3d(0,0,1.0));
        new_y.normalize();
        Eigen::Vector3d new_z = new_x.cross(new_y);
        R.row(0) = new_x;
        R.row(1) = new_y;
        R.row(2) = new_z;
    }
    return R;
}

Eigen::VectorXd BeamLinearElastic::create_global_self_weight() {
    Eigen::Vector3d w_G (0, -material_.Ax * material_.rho, 0);
    double L = beam_length_;

    Eigen::MatrixXd R = create_global_transformation_matrix();
    Eigen::VectorXd loads = Eigen::VectorXd::Zero(12);
    loads.segment(0, 3) = w_G * L / 2;
    loads.segment(6, 3) = w_G * L / 2;

    Eigen::Vector3d w_L = R * w_G;
    double LL = beam_length_ * beam_length_;
    Eigen::Vector3d M_L0(0, -w_L[2]*(LL)/12., w_L[1]*(LL)/12.);
    Eigen::Vector3d M_L1 = -M_L0;
    Eigen::MatrixXd RT = R.transpose();
    loads.segment(3, 3) = RT * M_L0;
    loads.segment(9, 3) = RT * M_L1;
    return loads;
}
Eigen::VectorXd BeamLinearElastic::compute_internal_force(Eigen::VectorXd &u)
{
    Eigen::MatrixXd R3 = create_global_transformation_matrix();
    Eigen::MatrixXd R = turn_diagblock(R3);
    Eigen::VectorXd Ru = R * u;
    Eigen::MatrixXd k = create_local_stiffness_matrix();
    return k * Ru;
}

Eigen::Vector2d BeamLinearElastic::compute_joint_stress(Eigen::VectorXd &u)
{
    Eigen::VectorXd f = compute_internal_force(u);
    Eigen::Vector2d stress;
    stress.setZero();
    for(int id = 0; id < 2; id++)
    {
        double Fx =     f(0 + id * 6);
        double Vy =     f(1 + id * 6);
        double Vz =     f(2 + id * 6);
        double T =      f(3 + id * 6);
        double Myy =    f(4 + id * 6);
        double Mzz =    f(5 + id * 6);

        double sgn = (id == 0? -1 : 1);

        double sigma_x = sgn * Fx / material_.Ax + abs(Myy) / material_.Sy + abs(Mzz) / material_.Sz;
        double tau_xy = abs(Vy) / material_.Asy + abs(T) / material_.C;
        double tau_xz = abs(Vz) / material_.Asz + abs(T) / material_.C;

        double vonmise = sqrt(sigma_x * sigma_x + 3 * (tau_xy * tau_xy + tau_xz * tau_xz));
        //double vonmise = abs(Fx) / material_.Ax;
        stress(id) = vonmise;
    }
    return stress;
}

}
