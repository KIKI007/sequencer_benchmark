//
// Created by 汪子琦 on 03.03.22.
//

#ifndef ROBO_CRAFT_BEAMLINEARELASTIC_H
#define ROBO_CRAFT_BEAMLINEARELASTIC_H
#include "Beam.h"
#include "BeamMaterial.h"
#include "Eigen/Sparse"

namespace frame
{

class BeamLinearElastic: public Beam{
public:

    BeamMaterial material_;

    bool frame_node_ = false;

public:

    BeamLinearElastic(const Beam& beam, const BeamMaterial &material)
    : Beam(beam), material_(material)
    {
        frame_node_ = true;
    }

    BeamLinearElastic(){
        frame_node_ = true;
    }

    BeamLinearElastic(const BeamLinearElastic &beam): Beam(beam), material_(beam.material_){
        frame_node_ = beam.frame_node_;
    }

public:

    /*
    local stiffness matrix for a bar with only axial force at two ends

        AE/L (u1 - u2) = F1
        AE/L (u2 - u1) = F2

    Virtual displacement (principle of virtual work) derivation:
    (same result can be derived using variational method or Galerkin method,
    which generalizes FEA to problems beyond structural mechanics.)

    We use linear (C0) interpolation between (x1, phi1) and ((x1, phi2))
    Thus the polynimial basis is [1, x]'
    The interpolation polynomial gives:
        [phi1, phi2]' = A [a0, a1]'
    where A = [[1, x1], [1, x2]]
    Inverting A gives:
        Ainv = (1/(x2-x1)) [[x2, -x1], [-1, 1]]
    Thus the *shape function* N:
        N = [(x2-x)/(x2-x1), (x-x1)/(x2-x1)]

    Thus, the displacement u can be interpolated over an element:
        u = N d
    where d is the nodal displacement dof of the element.

    Strains are determined from displacements:
        \epsilon = B d
    where [B] = [\partial][N] = d/dx [(L-x)/L, x/L]= [-1/L, L]

    Thus, the stiffness matrix
        [k] = \int_0^L [B]^T E [B] A dx = AE [[1, -1], [-1, 1]]

    Parameters
    ----------
    L : [type]
        bar element length
    A : [type]
        cross section area
    E : [type]
        Young's modulus
    Return
    ------
    K_ax_x : 2x2 numpy array
*/
    Eigen::Matrix2d axial_stiffness_matrix(double L, double A, double E);



    /*
        [Mx1, Mx2]' = ((GJ)/L) * [[1,-1], [-1,1]] [theta_x1, theta_x2]

        Parameters
        ----------
        L : [type]
            bar element length
        J : [type]
            torsional constant, unit: length unit^4
            In the case of circular, cylindrical shaft, it's equal to
            the polar moment of inertia of the cross section.
        G : [type]
            modulus of rigidity

        Return
        ------
        K_tor_x : 2x2 numpy array
    */
    Eigen::Matrix2d torsional_stiffness_matrix(double L, double J, double G);



    /*
     stiffness matrix of uniform beam element without the transverse shear deformation

     Here the stress-strain (\sigma ~ \epsilon) turns into

     bending moment-curvature (M ~ \kappa).

     strain e_x = - y/\kappa = - y (d^2 v / d^2 x)
     where \kappa is the radius of curvature

     Then from \sigma_x = E e_x,
     \sigma_x = - E y (d^2 v / d^2 x)

    From moment equilibrium,
        M_z = - \int_A \sigma_x y dA = E Iz (d^2 v / d^2 x)

                where the lateral displacement v(x) = [N]{d} and \kappa = [B] {d}
            nodal dof {d} = [v_1, theta_z1, v2, theta_z2]
                [B] = d^2/d^2 x [N]

                      We use cubic curve interpolation (C1) and [N]'s each row represents
                      the Langrange's interpolation functions (polynomials of deg 3 in this case).
                      Thus, the stiffness matrix [k] = int_0^L (E Iz d^2[N]) dx

     Parameters
     ----------
     L : [type]
        [description]
     E : [type]
        [description]
     Iz : [type]
        moment of inertia of the section about the z axis
        Iz = \int_A y^2 dA
     axis: int
        1 = local y axis , 2 = local z axis, default to 2
     Return
     ------
         K_bend_z : 4x4 numpy array
     */
    Eigen::Matrix4d bending_stiffness_matrix(double L, double E, double Iz, int axis=2);

    /*
    complete 12x12 stiffness matrix for a bisymmetrical member.

    Since for small displacements the axial force effects, torsion,
    and bending about each axis are uncoupled, the influence coeff
    **relating** these effects are zero.

    Parameters
    ----------
    L : float
        element length
    A : float
        cross section area
    Jx : float
        torsional constant, unit: length unit^4
        In the case of circular, cylindrical shaft, it's equal to
        the polar moment of inertia of the cross section.
    Iy : float
        moment of inertia \int_A z^2 dA
    Iz : float
        moment of inertia \int_A y^2 dA
    E : float
        Young's modulus
    mu : float
        Poisson ratio

    Returns
    -------
    K : 12x12 numpy array
    */

    Eigen::MatrixXd create_local_stiffness_matrix(bool reduce_bending = false);
    /*
     * Convert the local stiffness into the global coordinates
     */
    Eigen::MatrixXd create_global_stiffness_matrix(bool reduce_bending = false);

    Eigen::MatrixXd turn_diagblock(Eigen::MatrixXd R3);

    Eigen::VectorXd compute_internal_force(Eigen::VectorXd &u);

    Eigen::Vector2d compute_joint_stress(Eigen::VectorXd &u);

    /*
     * Compute Transformation
     */

    Eigen::MatrixXd create_global_transformation_matrix();

    /*
     * Compute global self weight
     */

    Eigen::VectorXd create_global_self_weight();

    void addToMatrix(std::vector<int> indexI, std::vector<int> indexJ, Eigen::MatrixXd &A, const Eigen::MatrixXd B);

    void addToMatrix(std::vector<int> indexI, std::vector<int> indexJ, std::vector<Eigen::Triplet<double>> &tri, const Eigen::MatrixXd B);


};

}


#endif  //ROBO_CRAFT_BEAMLINEARELASTIC_H
