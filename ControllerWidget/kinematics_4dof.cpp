#include "kinematics_4dof.h"

Kinematics_4DOF::Kinematics_4DOF(double L, double Rc, double Dknob)
    : m_L(L), m_Rc(Rc), m_dknob(Dknob)
{

}

Kinematics_4DOF::~Kinematics_4DOF()
{

}

void Kinematics_4DOF::operator =(const Kinematics_4DOF &Other)
{
   m_L = Other.m_L;
   m_Rc = Other.m_Rc;
   m_dknob = Other.m_dknob;
}

Eigen::Transform<double, 3, Eigen::Affine> Kinematics_4DOF::forwardKinematics(double gamma,
                                                                              double theta,
                                                                              double alpha,
                                                                              double d)
{
    if(alpha < 0)
    {
        alpha = abs(alpha); // flip sign
        theta = wrapToPi(theta + pi);
    }

    Eigen::Matrix4d t_;


    if(alpha > 0.000001)
    {
        t_ << sin(gamma + theta)*sin(theta) + cos(gamma + theta)*cos(theta)*cos(alpha),
             -sin(gamma + theta)*cos(theta) + cos(gamma + theta)*sin(theta)*cos(alpha),
                                                         cos(theta + gamma)*sin(alpha),
                                      (m_L*cos(theta + gamma)*(1. - cos(alpha)))/alpha,

             -cos(gamma + theta)*sin(theta) + sin(gamma + theta)*cos(theta)*cos(alpha),
              cos(gamma + theta)*cos(theta) + sin(gamma + theta)*sin(theta)*cos(alpha),
                                                         sin(theta + gamma)*sin(alpha),
                                      (m_L*sin(gamma + theta)*(1. - cos(alpha)))/alpha,

                                                                -cos(theta)*sin(alpha),
                                                                -sin(theta)*sin(alpha),
                                                                            cos(alpha),
                                                            d + (m_L*sin(alpha))/alpha,

                                                                            0, 0, 0, 1;
    }
    else
    {
        t_ << cos(gamma), -sin(gamma), 0,       0,
              sin(gamma),  cos(gamma), 0,       0,
                       0,           0, 1, d + m_L,
                       0,           0, 0,       1;
    }

    Eigen::Transform<double, 3, Eigen::Affine> T(t_);

    return T;
}

Eigen::Transform<double, 3, Eigen::Affine> Kinematics_4DOF::forwardKinematics(const Eigen::Vector4d &config)
{
    return forwardKinematics(config(0),config(1),config(2),config(3));
}

Eigen::Vector4d Kinematics_4DOF::inverseKinematics3D(const double xt,
                                                     const double yt,
                                                     const double zt,
                                                     const double gamma)
{
    double theta, alpha, d;
    // If gamma is not provided, this problem is underdefined. 1 DoF is free since we didn't specify
    // an orientation for the US crystal. We deal with this by setting default gamma to zero.
    if( (abs(xt) < 0.000001) && (abs(yt) < 0.000001) )
    {
        theta = 0.0; // bending axis 0
        alpha = 0.0; // bending angle 0
        d = zt - m_L; // translation
    }
    else
    {
        double gamma_plus_theta = atan2(yt, xt); // roll plus bending axis angle

        //std::cout << "atan2(1.0,0.0) = " << atan2(1.0,0.0) << std::endl;
        // Checking for this is not necessary, since atan2 returns pi/2 if x = 0
        //if(xt == 0)
        //    gamma_plus_theta = boost::math::copysign(pi/2., yt);

        theta = gamma_plus_theta - gamma;

        double df = sqrt(pow(xt,2) + pow(yt,2));

        double c = df/m_L; // constant

        //bool fZeroSuccess = fZeroAlpha(c, alpha);
        fZeroAlpha(c, alpha);

        d = zt - (m_L*sin(alpha))/alpha; // translation
    }

//    gamma = wrapToPi(gamma);
//    theta = wrapToPi(theta);

    // Calculate tip and final position vectors
    Eigen::Transform<double, 3, Eigen::Affine> Tcalc = forwardKinematics(gamma,theta,alpha,d);

    // Check if there is any error between given and calculated final position
    double tip_given = sqrt(pow(xt,2) + pow(yt,2) + pow(zt,2));
    //double tip_calc = sqrt( pow(Tcalc(0,3),2) + pow(Tcalc(1,3),2) + pow(Tcalc(2,3),2) );
    double tip_calc = Tcalc.matrix().col(3).segment(0,3).norm();
    double err = (tip_given - tip_calc)/tip_given;
    if(abs(err) > 0.000001)
        printf("BIG ERROR (%f) CHECK PARAMETERS\n", err);

//    if(true)
//    {
//        // Report to user
//        printf("Distance d is:\n\t %f mm\n", d);
//        printf("Angle gamma is:\n\t %f radians\n\t = %f degrees\n", gamma, gamma*180.0/pi);
//        printf("Angle theta is:\n\t %f radians\n\t = %f degrees\n", theta, theta*180.0/pi);
//        printf("Angle alpha is:\n\t %f radians\n\t = %f degrees\n", alpha, alpha*180.0/pi);
//    }

    return Eigen::Vector4d(gamma,theta,alpha,d);
}

Eigen::Vector4d Kinematics_4DOF::inverseKinematics(const Eigen::Transform<double, 3, Eigen::Affine> &T,
                                                   const double gamma)
{
    // get position from matrix
    double xt = T(0,3);
    double yt = T(1,3);
    double zt = T(2,3);

    double theta, alpha, d;

    if( (abs(xt) < 0.000001) && (abs(yt) < 0.000001) )
    {
        theta = 0; // bending axis 0
        alpha = 0; // bending angle 0
        d = zt - m_L; // translation
    }
    else
    {
        double gamma_plus_theta = atan2(yt, xt); // roll plus bending axis angle
        theta = gamma_plus_theta - gamma;

        bool fZeroSuccess;

        if (fmod(gamma_plus_theta,pi) == 0)
        {
            double c = xt/m_L/cos(gamma_plus_theta); // constant

            fZeroSuccess = fZeroAlpha(c, alpha);
        }
        else
        {
            double c = yt/m_L/sin(gamma_plus_theta); // constant

            fZeroSuccess = fZeroAlpha(c, alpha);
        }

        d = zt - (m_L*sin(alpha))/alpha; // translation

    }

    //gamma = wrapToPi(gamma);
    theta = wrapToPi(theta);

    // Calculate tip and final position vectors
    //Eigen::Transform<double, 3, Eigen::Affine> T_curr_calc = forwardKinematics(gamma,theta,alpha,d);

    return Eigen::Vector4d(gamma,theta,alpha,d);
}

Eigen::Matrix<double, 4, 2> Kinematics_4DOF::control_icra2016(const Eigen::Transform<double, 3, Eigen::Affine> &T,
                                                              const Eigen::Vector4d &dX,
                                                              double gamma)
{
    // Run inverse kinematics to get configuration space parameters
    Eigen::Vector4d configCurr = inverseKinematics(T, gamma);
    Eigen::Transform<double, 3, Eigen::Affine> T_Curr = forwardKinematics(configCurr);
    Eigen::Vector4d jointsCurr = configToJointSpace(configCurr);

    double xt = T_Curr(0,3) + dX(0);
    double yt = T_Curr(1,3) + dX(1);
    double zt = T_Curr(2,3) + dX(2);

    // Figure out angular deltas based on desired tip position
    // Keep the same amount of catheter base roll
    Eigen::Vector4d configTgt = inverseKinematics3D(xt, yt, zt,
                                                    configCurr(0)); // gammaCurr
    Eigen::Transform<double, 3, Eigen::Affine> T_Tgt = forwardKinematics(configTgt);
    //std::cout << "Target Tform\n" << T_Tgt.matrix() << std::endl;

    // Figure out how much to unroll
    // Project initial x axis to the final x-y plane
    Eigen::Vector3d x_init = T_Curr.rotation().block<3,1>(0,0);
    Eigen::Vector3d x_final = T_Tgt.rotation().block<3,1>(0,0);
    Eigen::Vector3d z_final = T_Tgt.rotation().block<3,1>(0,2);
    Eigen::Vector3d x_init_proj = x_init - x_init.dot(z_final)*z_final;
    x_init_proj.normalize();
    //std::cout << "x_init_proj\n" << x_init_proj << std::endl;

    // Find the angle between the projected and final x axes
    Eigen::Vector3d cross_xinit_xfinal = x_final.cross(x_init_proj);
    cross_xinit_xfinal.normalize();
    //std::cout << "cross_xinit_xfinal\n" << cross_xinit_xfinal << std::endl;
    double dot_xinitproj_xfinal = x_init_proj.dot(x_final);
    //std::cout << "dot_xinitproj_xfinal\n" << dot_xinitproj_xfinal << std::endl;
    // The dot product can be slightly larger than 1 or -1, causing acos to return NAN
    if( (-1.0 > dot_xinitproj_xfinal) && (dot_xinitproj_xfinal > -1.01))
        dot_xinitproj_xfinal = -1.0;
    if( (1.0 < dot_xinitproj_xfinal) && (dot_xinitproj_xfinal < 1.01) )
        dot_xinitproj_xfinal = 1.0;
    double delta_angle = acos(dot_xinitproj_xfinal);
    //std::cout << "delta_angle\n" << delta_angle << std::endl;
    if( z_final.dot(cross_xinit_xfinal) < 0.0)
        delta_angle = -delta_angle; // account for directionality
    //std::cout << "delta_angle\n" << delta_angle << std::endl;

    // Add the unroll and user's desired roll angles to handle roll
    double angle_diff = delta_angle + dX(3); // dX(3) =>> dpsi in MATLAB
    configTgt(0) = configTgt(0) + angle_diff; // (gamma) update handle roll angle
    configTgt(1) = configTgt(1) - angle_diff; // (theta) update bending axis angle

    // Get the target configuration space paramters and joint angles
    T_Tgt = forwardKinematics(configTgt); // update transform
    Eigen::Vector4d jointsTgt = configToJointSpace(configTgt);

    Eigen::Matrix<double, 4, 2> jointsCurrAndTgt;

    //std::cout << "Joints Curr\n" << jointsCurr << std::endl;
    //std::cout << "Joints Target\n" << jointsTgt << std::endl;

    jointsCurrAndTgt << jointsCurr, jointsTgt;

    return jointsCurrAndTgt;
}

Eigen::Matrix<double, 6, 4> Kinematics_4DOF::JacobianNumeric(double gamma,
                                                             double theta,
                                                             double alpha,
                                                             double d)
{
    double del_ = 0.0001; // small bump

    // vary d
    Eigen::Transform<double, 3, Eigen::Affine> T1plus = forwardKinematics(gamma,theta,alpha,d + del_);
    Eigen::Transform<double, 3, Eigen::Affine> T1minus = forwardKinematics(gamma,theta,alpha,d - del_);
    // vary gamma
    Eigen::Transform<double, 3, Eigen::Affine> T2plus = forwardKinematics(gamma + del_,theta,alpha,d);
    Eigen::Transform<double, 3, Eigen::Affine> T2minus = forwardKinematics(gamma - del_,theta,alpha,d);
    // vary theta
    Eigen::Transform<double, 3, Eigen::Affine> T3plus = forwardKinematics(gamma,theta + del_,alpha,d);
    Eigen::Transform<double, 3, Eigen::Affine> T3minus = forwardKinematics(gamma,theta - del_,alpha,d);
    // vary alpha
    Eigen::Transform<double, 3, Eigen::Affine> T4plus = forwardKinematics(gamma,theta,alpha + del_,d);
    Eigen::Transform<double, 3, Eigen::Affine> T4minus = forwardKinematics(gamma,theta,alpha - del_,d);

    // numerical x
    double delx1 = (T1plus(0,3) - T1minus(0,3))/(2*del_);
    double delx2 = (T2plus(0,3) - T2minus(0,3))/(2*del_);
    double delx3 = (T3plus(0,3) - T3minus(0,3))/(2*del_);
    double delx4 = (T4plus(0,3) - T4minus(0,3))/(2*del_);
    // numerical y
    double dely1 = (T1plus(1,3) - T1minus(1,3))/(2*del_);
    double dely2 = (T2plus(1,3) - T2minus(1,3))/(2*del_);
    double dely3 = (T3plus(1,3) - T3minus(1,3))/(2*del_);
    double dely4 = (T4plus(1,3) - T4minus(1,3))/(2*del_);
    // numerical z
    double delz1 = (T1plus(2,3) - T1minus(2,3))/(2*del_);
    double delz2 = (T2plus(2,3) - T2minus(2,3))/(2*del_);
    double delz3 = (T3plus(2,3) - T3minus(2,3))/(2*del_);
    double delz4 = (T4plus(2,3) - T4minus(2,3))/(2*del_);
    // numerical angle 1
    Eigen::Vector3d angles1Plus = T1plus.rotation().eulerAngles(2,1,0);
    Eigen::Vector3d angles1Minus = T1minus.rotation().eulerAngles(2,1,0);
    Eigen::Vector3d angles1 = (angles1Plus - angles1Minus)/(2*del_);
    // numerical angle 2
    Eigen::Vector3d angles2Plus = T2plus.rotation().eulerAngles(2,1,0);
    Eigen::Vector3d angles2Minus = T2minus.rotation().eulerAngles(2,1,0);
    Eigen::Vector3d angles2 = (angles2Plus - angles2Minus)/(2*del_);
    // numerical angle 3
    Eigen::Vector3d angles3Plus = T3plus.rotation().eulerAngles(2,1,0);
    Eigen::Vector3d angles3Minus = T3minus.rotation().eulerAngles(2,1,0);
    Eigen::Vector3d angles3 = (angles3Plus - angles3Minus)/(2*del_);
    // numerical angle 4
    Eigen::Vector3d angles4Plus = T4plus.rotation().eulerAngles(2,1,0);
    Eigen::Vector3d angles4Minus = T4minus.rotation().eulerAngles(2,1,0);
    Eigen::Vector3d angles4 = (angles4Plus - angles4Minus)/(2*del_);
    // numerical angles x
    double angx1 = angles1(2);
    double angx2 = angles2(2);
    double angx3 = angles3(2);
    double angx4 = angles4(2);
    // numerical angles y
    double angy1 = angles1(1);
    double angy2 = angles2(1);
    double angy3 = angles3(1);
    double angy4 = angles4(1);
    // numerical angles z
    double angz1 = angles1(0);
    double angz2 = angles2(0);
    double angz3 = angles3(0);
    double angz4 = angles4(0);

    Eigen::Matrix<double, 6, 4> J;
    J << delx1, delx2, delx3, delx4,
         dely1, dely2, dely3, dely4,
         delz1, delz2, delz3, delz4,
         angx1, angx2, angx3, angx4,
         angy1, angy2, angy3, angy4,
         angz1, angz2, angz3, angz4;

    return J;
}

Eigen::Vector4d Kinematics_4DOF::dampedLeastSquaresStep(const Eigen::Matrix<double, 6, 4> &J,
                                                        const Eigen::Matrix<double, 6, 1> &C_error)
{
    // calculate the condition number
    Eigen::Matrix<double, 4, 4> We4 = Eigen::Matrix<double, 4, 4>::Identity() * (1./4.); // weighting matrix
    Eigen::Matrix<double, 6, 6> We6 = Eigen::Matrix<double, 6, 6>::Identity() * (1./6.); // weighting matrix
    double kap = sqrt( (J*We4*J.transpose()).trace() * (J.transpose()*We6*J).trace() ); // Weighted Frobenius norm
    //double kap = sqrt( (J*J.transpose()).trace() * (J.transpose*J).trace() ); // Frobenius norm
    double condNum = 1./kap; // condition number
    printf("J cond = %.3f\n", condNum);

    // lambda = 0.2;
    double lambda = condNum;
    Eigen::Matrix<double, 6, 6> mtrx = (J * J.transpose() + Eigen::Matrix<double, 6, 6>::Identity() * lambda);

    // Eigen::JacobiSVD<MatrixXf> svd(J, ComputeThinU | ComputeThinV);
    // Eigen::Matrix<double, 6, 4> E;
    // E(0,0) = svd.singularValues()(0) / (svd.singularValues()(0)*svd.singularValues()(0) + kap*kap);
    // E(1,1) = svd.singularValues()(1) / (svd.singularValues()(1)*svd.singularValues()(1) + kap*kap);
    // E(2,2) = svd.singularValues()(2) / (svd.singularValues()(2)*svd.singularValues()(2) + kap*kap);
    // E(3,3) = svd.singularValues()(3) / (svd.singularValues()(3)*svd.singularValues()(3) + kap*kap);
    // Eigen::Matrix<double, 4, 6> VEUT = svd.matrixV() * E * svd.matrixU().transpose();
    // Eigen::Vector4d delta_C = VEUT * C_error;

    //Eigen::Vector4d delta_C = J.transpose()* mtrx.inverse() * C_error;
    Eigen::Vector4d delta_C = J.transpose()* mtrx.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(C_error);
    return delta_C;
}

// taskSpace = {x,y,z,psi}
// configSpace = {gamma,theta,alpha,d}
Eigen::Vector4d Kinematics_4DOF::taskToConfigSpace(const Eigen::Vector4d &taskSpace)
{
    double xt = taskSpace(0), yt = taskSpace(1), zt = taskSpace(2), psi = taskSpace(3);
    double gamma, theta, alpha, d;

    if( (abs(xt) < 0.000001) && (abs(yt) < 0.000001) )
    {
        alpha = 0;
        theta = 0;
        gamma = psi;
        d = zt - m_L;
    }
    else
    {
        double gamma_plus_theta = atan2(yt, xt); // roll plus bending axis angle
        theta = psi;
        gamma =  gamma_plus_theta - psi;

        bool fZeroSuccess;

        if (fmod(gamma_plus_theta,pi) == 0)
        {
            double c = xt/m_L/cos(gamma_plus_theta); // constant

            fZeroSuccess = fZeroAlpha(c, alpha);
        }
        else
        {
            double c = yt/m_L/sin(gamma_plus_theta); // constant

            fZeroSuccess = fZeroAlpha(c, alpha);
        }

        d = zt - (m_L*sin(alpha))/alpha; // translation

    }

    // gamma = wrapToPi(gamma);
    // theta = wrapToPi(theta);
    if(alpha < 0)
        printf("\n\n\n\n\n ALPHA LESS THAN ZERO!!! \n\n\n\n\n");

    return Eigen::Vector4d(gamma,theta,alpha,d);
}

// {gamma,theta,alpha,d}
Eigen::Vector4d Kinematics_4DOF::configToTaskSpace(const Eigen::Vector4d &configSpace)
{
    double gamma = configSpace(0), theta = configSpace(1), alpha = configSpace(2), d = configSpace(3);
    double x,y,z,psi;

    if(abs(alpha) < 0.0001)
    {
        x = 0;
        y = 0;
        z = d + m_L;
        psi = gamma;
    }
    else
    {
        double r = m_L*(1.0-cos(alpha))/alpha;
        x = r*cos(gamma + theta);
        y = r*sin(gamma + theta);
        z = d + m_L*sin(alpha)/alpha;
        psi = theta;
    }

    return Eigen::Vector4d(x,y,z,psi);
}

Eigen::Matrix4d Kinematics_4DOF::JacobianNumericTaskSpace(const Eigen::Vector4d &configCurr)
{
    double gamma = configCurr(0), theta = configCurr(1), alpha = configCurr(2), d = configCurr(3);

    double del_ = 0.0001; // small bump

    // vary d
    Eigen::Transform<double, 3, Eigen::Affine> T1plus = forwardKinematics(gamma,theta,alpha,d + del_);
    Eigen::Transform<double, 3, Eigen::Affine> T1minus = forwardKinematics(gamma,theta,alpha,d - del_);
    // psi1plus = theta;
    // psi1minus = theta;

    // vary gamma
    Eigen::Transform<double, 3, Eigen::Affine> T2plus = forwardKinematics(gamma + del_,theta,alpha,d);
    Eigen::Transform<double, 3, Eigen::Affine> T2minus = forwardKinematics(gamma - del_,theta,alpha,d);
    // psi2plus = theta;
    // psi2minus = theta;

    // vary theta
    Eigen::Transform<double, 3, Eigen::Affine> T3plus = forwardKinematics(gamma,theta + del_,alpha,d);
    Eigen::Transform<double, 3, Eigen::Affine> T3minus = forwardKinematics(gamma,theta - del_,alpha,d);
    double psi3plus = theta + del_;
    double psi3minus = theta - del_;

    // vary alpha
    Eigen::Transform<double, 3, Eigen::Affine> T4plus = forwardKinematics(gamma,theta,alpha + del_,d);
    Eigen::Transform<double, 3, Eigen::Affine> T4minus = forwardKinematics(gamma,theta,alpha - del_,d);
    // psi4plus = theta;
    // psi4minus = theta;

    // numerical x
    double delx1 = (T1plus(0,3) - T1minus(0,3))/(2*del_);
    double delx2 = (T2plus(0,3) - T2minus(0,3))/(2*del_);
    double delx3 = (T3plus(0,3) - T3minus(0,3))/(2*del_);
    double delx4 = (T4plus(0,3) - T4minus(0,3))/(2*del_);
    // numerical y
    double dely1 = (T1plus(1,3) - T1minus(1,3))/(2*del_);
    double dely2 = (T2plus(1,3) - T2minus(1,3))/(2*del_);
    double dely3 = (T3plus(1,3) - T3minus(1,3))/(2*del_);
    double dely4 = (T4plus(1,3) - T4minus(1,3))/(2*del_);
    // numerical z
    double delz1 = (T1plus(2,3) - T1minus(2,3))/(2*del_);
    double delz2 = (T2plus(2,3) - T2minus(2,3))/(2*del_);
    double delz3 = (T3plus(2,3) - T3minus(2,3))/(2*del_);
    double delz4 = (T4plus(2,3) - T4minus(2,3))/(2*del_);
    // numerical psi
    // psi1,2,4 = 0
    double psi3 = (psi3plus - psi3minus)/(2*del_);

    Eigen::Matrix4d J;
    if(abs(alpha) < 0.0001){
        J << 1e-12, delx3, delx4, delx1,
             dely2, 1e-12, dely4, dely1,
             delz2, delz3, delz4, delz1,
              psi3,   0.0,   0.0,   0.0;
    }
    else{
        J << delx2, delx3, delx4, delx1,
             dely2, dely3, dely4, dely1,
             delz2, delz3, delz4, delz1,
               0.0,  psi3,   0.0,   0.0;
    }

    return J;
}

Eigen::Vector4d Kinematics_4DOF::JacobianStep(const Eigen::Vector4d &currTask, const Eigen::Vector4d &targetTask, const Eigen::Vector4d &currConfig)
{
    Eigen::Vector4d dxyzpsi = targetTask - currTask;
    double normXYZ = dxyzpsi.segment<3>(0).norm(), errorRot = abs(wrapToPi(abs(dxyzpsi(3))));
    double euclThresh = 1.5;
    double rotThresh = pi/5;

    Eigen::Vector4d delta_C; delta_C = Eigen::Vector4d::Zero();
    Eigen::Vector4d newConfig = currConfig;
    Eigen::Vector4d newTask = currTask;

    unsigned int iter = 0;
    bool isNotConverged = (normXYZ > 0.01) || (errorRot > 0.01);
    while( isNotConverged && (iter < 30) )
    {
        // clamp error

        // don't translate more than 1.5 at a time
        if(normXYZ > euclThresh)
            dxyzpsi.segment<3>(0) *= euclThresh/normXYZ;
        // don't rotate more than 36 degrees at a time
        if(abs(dxyzpsi(3)) > rotThresh)
            dxyzpsi(3) = boost::math::copysign(rotThresh, dxyzpsi(3));

        // numerically compute Jacobian
        Eigen::Matrix4d J = JacobianNumericTaskSpace(newConfig);

        // J_pseudoinverseWithNullspace
        Eigen::Vector4d v_(0.5,0.5,0.5,0.5); v_ *= 2.0;
        Eigen::JacobiSVD<Eigen::Matrix4d> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
        //Eigen::Matrix4d pinvJ = svd.solve(Eigen::Matrix4d::Identity()); // J.inverse();
        //delta_C = pinvJ * dxyzpsi + (Eigen::Matrix4d::Identity() - pinvJ*J)*v_;
        delta_C = svd.solve(dxyzpsi) + (Eigen::Matrix4d::Identity() - svd.solve(J))*v_;

        newConfig += delta_C;

        if(newConfig(2) < 0.0) // gamma, theta, alpha, d
        {
            newConfig(2) = abs(newConfig(2));
            newConfig(1) = wrapToPi(newConfig(1) + pi);
        }

        newTask = configToTaskSpace(newConfig);

        dxyzpsi = targetTask - newTask;
        normXYZ = dxyzpsi.segment<3>(0).norm();
        errorRot = abs(wrapToPi(abs(dxyzpsi(3))));

        isNotConverged = (normXYZ > 0.01) || (errorRot > 0.01);

        iter++;
    }

    if(isNotConverged)
    {
        printf("Not converged, %.3f %.3f.\n", normXYZ, errorRot);
        newConfig = currConfig;
    }
    else
        printf("Converged in %d steps.\n", iter);

    return newConfig;
}

double Kinematics_4DOF::wrapToPi(double lambda)
{
    double signedPI = boost::math::copysign(pi, lambda);
    return lambda = fmod(lambda + signedPI,(2*pi)) - signedPI;
}

bool Kinematics_4DOF::fZeroAlpha(const double c, double &alpha)
{
    // Newton's Method - init
    double a0 = 2*c; // initial guess
    double a1 = 0.0, y, yprime;
    bool foundSoln = false;

    // Newton's Method - iterate
    for(size_t i = 0; i < 25; i++)
    {
        y = 1.0 - cos(a0) - a0*c; // function
        yprime = sin(a0) - c;     // 1st derivative

        if(abs(yprime) < 1.0e-14)
            break;

        a1 = a0 - y/yprime;

        if(abs(a1 - a0) <= (1.0e-7 * abs(a1)) )
        {
            printf("Found solution in %d steps.\n", i+1);
            foundSoln = true;
            break;
        }

        a0 = a1 ;
    }
    // Newton's Method - end

    if(!foundSoln)
        printf("fzero failed!!!\n");

    alpha = a1;

    return foundSoln;
}

/*!
 * @param[in]  d        Translation
 * @param[in]  gamma    Roll angle
 * @param[in]  theta    Bending axis angle
 * @param[in]  alpha    Bending angle
 * @param[in]  Rc       Catheter radius
 * @param[in]  dknob    Diameter of the knob
 * @param[out] outputs  A vector that contains the following values:
 *                            phi_trans : Translation
 *                            phi_pitch : Roll angle
 *                            phi_yaw   : Bending axis angle
 *                            phi_roll  : Bending angle
 */
void Kinematics_4DOF::configToJointSpace(const double gamma,
                                         const double theta,
                                         const double alpha,
                                         const double d,
                                         Eigen::Vector4d &outputs)
{
    //  Paul added a negative in front of this to be consistent with the way yaw is defined
    //  for the catheter absolute joint angles
    outputs(0) = d;
    outputs(1) = 2.0 * ( m_Rc * alpha * cos(theta)) / m_dknob;
    outputs(2) = 2.0 * (-m_Rc * alpha * sin(theta)) / m_dknob;
    outputs(3) = gamma;
}

void Kinematics_4DOF::configToJointSpace(const Eigen::Vector4d &inputs, Eigen::Vector4d &outputs)
{
    configToJointSpace(inputs(0),inputs(1),inputs(2),inputs(3),outputs);
}

Eigen::Vector4d Kinematics_4DOF::configToJointSpace(const Eigen::Vector4d &inputs)
{
    Eigen::Vector4d outputs;
    configToJointSpace(inputs, outputs);
    return outputs;
}
