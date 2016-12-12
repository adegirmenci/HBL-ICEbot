#ifndef FILTFILT_H
#define FILTFILT_H

#include <vector>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <Eigen/Dense>

#include <boost/math/special_functions/sinc.hpp>

#include "kinematics_4dof.h" // for pi


//#include <spuce/filters/design_window.h>
//#include <spuce/filters/design_fir.h>
//#include <spuce/typedefs.h>
//#include <spuce/filters/fir_coeff.h>
//#include <spuce/filters/window.h>
//#include <spuce/filters/fir.h>

//#include <FIR.h>

// Adapted from:
// http://stackoverflow.com/questions/17675053/matlabs-filtfilt-algorithm/27270420#27270420

class filtfilt
{
public:
    filtfilt();
    ~filtfilt();

    // X is the original data, Y is the output
    void run(const std::vector<double> &X, std::vector<double> &Y);

    void setFilterCoefficients(const std::vector<double> &B, const std::vector<double> &A);

private:
    void updateFilterParameters();

    // helper functions
    void filter(const std::vector<double> &X, std::vector<double> &Y, std::vector<double> &Zi);
    void add_index_range(std::vector<int> &indices, int beg, int end, int inc = 1);
    void add_index_const(std::vector<int> &indices, int value, size_t numel);
    void append_vector(std::vector<double> &vec, const std::vector<double> &tail);
    std::vector<double> subvector_reverse(const std::vector<double> &vec, int idx_end, int idx_start);
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

};

#endif // FILTFILT_H
