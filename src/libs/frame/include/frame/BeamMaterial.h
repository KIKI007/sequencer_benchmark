//
// Created by 汪子琦 on 04.03.22.
//

#ifndef ROBO_CRAFT_BEAMMATERIAL_H
#define ROBO_CRAFT_BEAMMATERIAL_H
#include <iostream>
#include <cmath>
namespace frame
{
enum CrossSectionType {None, ROUND, SQUARE};

struct BeamMaterial {
    CrossSectionType type;
    double width; //cross section width
    double Ax;    //cross section area (m^2)
    double Jxx;   //torsional constant (m^4)
    double Iyy;   //moment of inertia y (m^4)
    double Izz;   //moment of inertia z (m^4)
    double E;    //Young's modulus (Pa)
    double mu;   //Poisson ratio
    double rho;  //density (N / m^3)
    double Sy;   //section area
    double Sz;   //section area
    double C;    //Torsion Shear Constant
    double Asy;  //section shear area
    double Asz;  //section shear area

public:
    BeamMaterial(){
        type = None;
        width = Ax = Jxx = Iyy = Izz = E = mu = rho = 0.0;
    }

    BeamMaterial(CrossSectionType type, double width, double E, double mu, double rho)
    :E(E), mu(mu), rho(rho), type(type), width(width){
        computeProfile();
    }

public:

    void computeProfile(){
        switch (type) {
            case ROUND:
                setRoundProfile(width, E, mu, rho);
                break;
            case SQUARE:
                setSquareProfile(width, E, mu, rho);
                break;
            default:
                setDefaultProfile(Ax, E, mu, rho);
        }
    }

private:

    void setRoundProfile(double radius, double E, double mu, double rho){
        Ax = M_PI * std::pow(radius, 2.0);
        Iyy = M_PI / 4. * std::pow(radius, 4.0);
        Izz = M_PI / 4. * std::pow(radius, 4.0);
        Jxx = M_PI / 2. * std::pow(radius, 4.0);
        Sy = Sz = Iyy / radius;
        C = Jxx / radius;
        Asy = Asz = Ax * 0.5;
    }

    void setSquareProfile(double width, double E, double mu, double rho){
        Ax = std::pow(width, 2.0);
        Iyy = 1.0 / 12. * std::pow(width, 4.0);
        Izz = 1.0 / 12. * std::pow(width, 4.0);
        Jxx = 1.0 / 16. * std::pow(width, 4.0);
        Sy = Sz = Iyy / (width/2);
        C = pow(width, 3) / 4;
        Asy = Asz = Ax * 0.45;
    }

    void setDefaultProfile(double A, double E, double mu, double rho)
    {
        width = sqrt(A / M_PI);
        Sy = Sz = Iyy / width;
        C = Jxx / width;
        Asy = Asz = Ax * 0.5;
    }
};
}
#endif  //ROBO_CRAFT_BEAMMATERIAL_H
