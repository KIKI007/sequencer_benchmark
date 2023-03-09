//
// Created by ziqwang on 02.08.21.
//

#ifndef TIMBER_JOINT_BEAM_H
#define TIMBER_JOINT_BEAM_H

#include "BeamMaterial.h"
#include "Eigen/Dense"
#include <vector>
#define FLOAT_ERROR_LARGE 1E-4
#define FLOAT_ERROR_SMALL 1E-6
namespace frame {
////////////////////////////////////////////
// Box With Coordinate Frame
////////////////////////////////////////////
class Frame {
public:
    typedef Eigen::Matrix<double, 3, 1> Vector3;

    Vector3 origin, xaxis, yaxis, zaxis;

public:
    Frame() {
        origin = Vector3(0, 0, 0);
        xaxis = Vector3(1, 0, 0);
        yaxis = Vector3(0, 1, 0);
        zaxis = Vector3(0, 0, 1);
    }

    Frame(Vector3 _origin, Vector3 _xaxis, Vector3 _yaxis, Vector3 _zaxis) {
        origin = _origin;
        xaxis = _xaxis;
        yaxis = _yaxis;
        zaxis = _zaxis;
    }

public:
    Vector3 convertGlobalToLocal(Vector3 globalPt) {
        double u = (globalPt - origin).dot(xaxis);
        double v = (globalPt - origin).dot(yaxis);
        double w = (globalPt - origin).dot(zaxis);
        return Vector3(u, v, w);
    }

    Vector3 convertLocalToGlobal(Vector3 localPt) {
        return origin + localPt.x() * xaxis + localPt.y() * yaxis + localPt.z() * zaxis;
    }

    void fromZAxis(Vector3 _zaxis) {
        origin = Vector3(0, 0, 0);
        zaxis = _zaxis.normalized();
        xaxis = zaxis.template cross(Vector3(1, 0, 0));
        if (xaxis.norm() < FLOAT_ERROR_LARGE) {
            xaxis = zaxis.template cross(Vector3(0, 1, 0));
        }
        xaxis = xaxis.normalized();
        yaxis = zaxis.cross(xaxis).normalized();
    }

    void transform(Vector3 rotVec, Vector3 transVec) {
        origin += transVec;
        xaxis = Eigen::AngleAxisd(rotVec.norm(), rotVec.normalized()) * xaxis;
        yaxis = Eigen::AngleAxisd(rotVec.norm(), rotVec.normalized()) * yaxis;
        zaxis = Eigen::AngleAxisd(rotVec.norm(), rotVec.normalized()) * zaxis;
    }

    void transform(Eigen::Matrix<double, 6, 1> rigidVec) {
        transform(rigidVec.segment(3, 3), rigidVec.segment(0, 3));
    }

    Eigen::Matrix<double, 3, 3> matrixR() {
        Eigen::Matrix<double, 3, 3> R;
        R.col(0) = zaxis;
        R.col(1) = xaxis;
        R.col(2) = yaxis;
        return R;
    }
};

class Beam {
public:
    //central line's coordinate frame
    //the origin point of the frame is the center of the bottom face
    Frame frame_;

    double beam_length_;

    double beam_width_;

    double beam_fixture_ = 0;

    int partID_;

    CrossSectionType type_;

public:
    Beam() {}

    Beam(Frame frame, double beam_length, double beam_width) {
        partID_ = -1;

        frame_ = frame;
        beam_length_ = beam_length;
        beam_width_ = beam_width;
        beam_fixture_ = 0;
    }

    Beam(const Beam &beam) {
        partID_ = beam.partID_;

        frame_ = beam.frame_;
        beam_length_ = beam.beam_length_;
        beam_width_ = beam.beam_width_;
        beam_fixture_ = beam.beam_fixture_;
        type_ = beam.type_;
    }

public:
    virtual int getMesh(Eigen::MatrixXd &V, Eigen::MatrixXi &F) const {
        return 0;
    }

    double computeCenterY() {
        return (frame_.origin + frame_.zaxis * beam_length_ / 2).y();
    }

    Eigen::Vector3d center(){
        return frame_.origin + frame_.zaxis * beam_length_ / 2;
    }

    virtual void computeMaterialProperty(double &A, double &Iy, double &Iz, double &Jx) {
        A = 0;
        Iy = 0;
        Iz = 0;
        Jx = 0;
    }
};

class BeamRound : public Beam {
protected:
    const int num_circle_sample_ = 32;

public:
    BeamRound() {
        type_ = ROUND;
    }

    BeamRound(Frame frame, double beam_length, double beam_width) : Beam(frame, beam_length, beam_width) {
        type_ = ROUND;
    }

    BeamRound(Eigen::Vector3d endu, Eigen::Vector3d endv, double radius);

public:
    int getMesh(Eigen::MatrixXd &V, Eigen::MatrixXi &F) const override;
};

class BeamSquare : public Beam {
public:
    BeamSquare() {
        type_ = SQUARE;
    }

    BeamSquare(Frame frame, double beam_length, double beam_width) : Beam(frame, beam_length, beam_width) {
        type_ = SQUARE;
    }

public:
    int getMesh(Eigen::MatrixXd &V, Eigen::MatrixXi &F) const override;

public:
    bool checkPtInside(Eigen::Vector3d pt);
};
}
#endif //TIMBER_JOINT_BEAM_H
