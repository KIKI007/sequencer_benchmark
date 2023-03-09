//
// Created by ziqwang on 02.08.21.
//

#include "frame/Beam.h"

namespace frame {

/*******************************************************************************************************************************
 *                                                      BeamSquare
 ********************************************************************************************************************************/

int BeamSquare::getMesh(Eigen::MatrixXd &V, Eigen::MatrixXi &F) const {
    Eigen::MatrixXi QF = Eigen::MatrixXi(6, 4);
    QF << 0, 3, 2, 1, 4, 5, 6, 7, 0, 1, 5, 4, 1, 2, 6, 5, 2, 3, 7, 6, 0, 4, 7, 3;

    F = Eigen::MatrixXi(12, 3);
    for (int id = 0; id < 6; id++) {
        F.row(id * 2) = Eigen::Vector3i(QF(id, 0), QF(id, 1), QF(id, 2));
        F.row(id * 2 + 1) = Eigen::Vector3i(QF(id, 2), QF(id, 3), QF(id, 0));
    }

    V = Eigen::MatrixXd(8, 3);

    std::vector<double> deltaX = {-0.5, 0.5, 0.5, -0.5, -0.5, 0.5, 0.5, -0.5};
    std::vector<double> deltaY = {-0.5, -0.5, 0.5, 0.5, -0.5, -0.5, 0.5, 0.5};
    std::vector<double> deltaZ = {0, 0, 0, 0, 1, 1, 1, 1};

    for (int id = 0; id < deltaX.size(); id++) {
        Eigen::Vector3d pt =
            frame_.origin + frame_.xaxis * deltaX[id] * beam_width_ + frame_.yaxis * deltaY[id] * beam_width_ + frame_.zaxis * deltaZ[id] * beam_length_;
        V.row(id) = pt.transpose();
    }

    return F.rows();
}
bool BeamSquare::checkPtInside(Eigen::Vector3d pt) {
    Eigen::Vector3d localPt = frame_.convertGlobalToLocal(pt);

    //beam's cross section width

    if (std::abs(localPt.x()) < beam_width_ / 2 + FLOAT_ERROR_LARGE && std::abs(localPt.y()) < beam_width_ / 2 + FLOAT_ERROR_LARGE &&
        -FLOAT_ERROR_LARGE <= localPt.z() && localPt.z() <= beam_length_ + FLOAT_ERROR_LARGE) {
        return true;
    } else {
        return false;
    }
}

/*******************************************************************************************************************************
 *                                                      BeamRound
 ********************************************************************************************************************************/

BeamRound::BeamRound(Eigen::Vector3d endu, Eigen::Vector3d endv, double radius) {
    Eigen::Vector3d zaxis = endv - endu;
    zaxis.normalize();

    Eigen::Vector3d xaxis = Eigen::Vector3d(1, 0, 0).cross(zaxis);
    if (xaxis.norm() < FLOAT_ERROR_LARGE) {
        xaxis = Eigen::Vector3d(0, 1, 0).cross(zaxis);
    }
    xaxis.normalize();
    Eigen::Vector3d yaxis = zaxis.cross(xaxis);

    frame_.origin = endu;
    frame_.zaxis = zaxis;
    frame_.xaxis = xaxis;
    frame_.yaxis = yaxis;

    beam_length_ = (endv - endu).norm();
    beam_width_ = radius;
}

int BeamRound::getMesh(Eigen::MatrixXd &V, Eigen::MatrixXi &F) const {
    V = Eigen::MatrixXd(2 * num_circle_sample_, 3);
    F = Eigen::MatrixXi(2 * num_circle_sample_, 3);

    for (int kd = 0; kd < num_circle_sample_; kd++) {
        double angle = (double)kd / num_circle_sample_ * M_PI * 2;
        Eigen::Vector3d pt = frame_.origin + (cos(angle) * frame_.xaxis + sin(angle) * frame_.yaxis) * beam_width_ / 2;
        V.row(kd) = pt;
        V.row(kd + num_circle_sample_) = pt + frame_.zaxis * beam_length_;
    }

    for (int kd = 0; kd < num_circle_sample_; kd++) {
        int next_kd = (kd + 1) % num_circle_sample_;
        F.row(2 * kd) = Eigen::Vector3i(kd, next_kd, next_kd + num_circle_sample_);
        F.row(2 * kd + 1) = Eigen::Vector3i(kd, next_kd + num_circle_sample_, kd + num_circle_sample_);
    }

    return F.rows();
}
}