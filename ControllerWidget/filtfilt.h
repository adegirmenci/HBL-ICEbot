#ifndef FILTFILT_H
#define FILTFILT_H

#include <vector>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <Eigen/Dense>
#include <Eigen/StdDeque>

#include <boost/math/special_functions/sinc.hpp>

#include "kinematics_4dof.h" // for pi

#define FILTER_ORDER 50            // FILTERORDER
#define SAMPLE_DELTA_TIME 0.005992 // SAMPLE_TIME
#define HEART_RATE 120             // HEARTRATE

//#include <spuce/filters/design_window.h>
//#include <spuce/filters/design_fir.h>
//#include <spuce/typedefs.h>
//#include <spuce/filters/fir_coeff.h>
//#include <spuce/filters/window.h>
//#include <spuce/filters/fir.h>

//#include <FIR.h>

// Adapted from:
// http://stackoverflow.com/questions/17675053/matlabs-filtfilt-algorithm/27270420#27270420

typedef Eigen::Transform<double,3,Eigen::Affine>    EigenAffineTransform3d;
typedef             Eigen::Matrix<double, 7 , 1>    EigenVector7d;
// in order to use deque with Eigen Transform, we need this typedef
typedef std::deque< EigenAffineTransform3d, Eigen::aligned_allocator<EigenAffineTransform3d> > EigenDequeAffineTform3d;
typedef std::deque<          EigenVector7d,          Eigen::aligned_allocator<EigenVector7d> > EigenDequeVector7d;

class filtfilt
{
public:
    filtfilt();
    ~filtfilt();

    // X is the original data, Y is the output
    void run(const std::vector<double> &X, std::vector<double> &Y);
    //void run(const std::deque<double> &X, std::deque<double> &Y);
    void run(const EigenDequeVector7d &X, EigenDequeVector7d &Y);

    // TODO : add function to change the heartrate, sampling freq, and maybe filter order
    void setFilterCoefficients(const std::vector<double> &B, const std::vector<double> &A);

private:
    void updateFilterParameters();

    // helper functions
    void filter(const std::vector<double> &X, std::vector<double> &Y, std::vector<double> &Zi);
    void filter(const EigenDequeVector7d &X, EigenDequeVector7d &Y, EigenDequeVector7d &Zi);

    void add_index_range(std::vector<int> &indices, int beg, int end, int inc = 1);

    void add_index_const(std::vector<int> &indices, int value, size_t numel);

    void append_vector(std::vector<double> &vec, const std::vector<double> &tail);
    void append_vector(EigenDequeVector7d &vec, const EigenDequeVector7d &tail);

    std::vector<double> subvector_reverse(const std::vector<double> &vec, int idx_end, int idx_start);
    EigenDequeVector7d subvector_reverseEig(const EigenDequeVector7d &vec, int idx_end, int idx_start);

    inline int max_val(const std::vector<int>& vec)
    {
        return std::max_element(vec.begin(), vec.end())[0];
    }

    std::vector<double> FIR_LPF_LeastSquares(size_t L, const double deltaT);
    std::vector<double> genHammingWindow(const size_t L);
    std::vector<double> calcWindow(size_t m, size_t  n);

    // data members

    std::vector<double> m_A, m_B; // filter coefficients
    int m_na;
    int m_nb;
    int m_nfilt;
    int m_nfact;
    std::vector<int> m_rows, m_cols;
    std::vector<double> m_data;
    Eigen::MatrixXd m_zzi;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif // FILTFILT_H
