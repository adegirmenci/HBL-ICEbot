#ifndef KINEMATICS_4DOF_H
#define KINEMATICS_4DOF_H

#include <math.h>

//#include <chrono>
//#include <thread>
#include <iostream>

#include <boost/math/constants/constants.hpp>
#include <boost/math/special_functions/sign.hpp>
//#include <boost/math/tools/roots.hpp>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

/*!
 *  \brief     Four degree of freedom catheter kinematics class.
 *  \author    Alperen Degirmenci
 *  \version   1.0alpha
 *  \date      2015-2016
 */
class Kinematics_4DOF
{
public:
    Kinematics_4DOF(double L  = 1.0, double Rc = 1.0, double Dknob = 1.0);

    void operator = (const Kinematics_4DOF &Other); // assignment operator definition

    void configToJointSpace(const double gamma,
                            const double theta,
                            const double alpha,
                            const double d,
                            Eigen::Vector4d &outputs);

    void configToJointSpace(const Eigen::Vector4d &inputs,
                            Eigen::Vector4d &outputs);

    Eigen::Vector4d configToJointSpace(const Eigen::Vector4d &inputs);

    Eigen::Transform<double, 3, Eigen::Affine> forwardKinematics(double gamma,
                                                                 double theta,
                                                                 double alpha,
                                                                 double d);

    Eigen::Transform<double, 3, Eigen::Affine> forwardKinematics(const Eigen::Vector4d &config);

    Eigen::Vector4d inverseKinematics3D(const double xt,
                                        const double yt,
                                        const double zt,
                                        const double gamma = 0.0);

    Eigen::Vector4d inverseKinematics(const Eigen::Transform<double, 3, Eigen::Affine> &T,
                                      const double gamma); // 4 DOF

    Eigen::Matrix<double, 4, 2> control_icra2016(const Eigen::Transform<double, 3, Eigen::Affine> &T,
                                const Eigen::Vector4d &dX,
                                double gamma);

    Eigen::Matrix<double, 6, 4> JacobianNumeric(double gamma,
                                                  double theta,
                                                  double alpha,
                                                  double d);

    Eigen::Vector4d dampedLeastSquaresStep(const Eigen::Matrix<double, 6, 4> &J,
                                           const Eigen::Matrix<double,6,1> &C_error);

    const double pi = boost::math::constants::pi<double>();

private:
    // GEOMETRY CONSTANTS
    double m_L; // constant length of distal bending section, for all ICE
    double m_Rc; // radius of catheter
    double m_dknob; // diameter of pulley inside the handle knob

    // STATE
    Eigen::Vector4d m_state;


    // Helper functions
    double wrapToPi(double lambda);

    bool fZeroAlpha(const double c, double &alpha);

    // time since last loop
    // time between EM readings

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif // KINEMATICS_4DOF_H
