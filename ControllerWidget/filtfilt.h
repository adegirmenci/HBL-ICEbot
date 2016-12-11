#ifndef FILTFILT_H
#define FILTFILT_H

#include <vector>
#include <algorithm>
#include <iostream>
#include <Eigen/Dense>


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

    // A and B are the coefficients, X is the original data, Y is the output
    void run(std::vector<double> B, std::vector<double> A, const std::vector<double> &X, std::vector<double> &Y);

    std::vector<double> m_A, m_B;

private:
    void add_index_range(std::vector<int> &indices, int beg, int end, int inc);

    void add_index_const(std::vector<int> &indices, int value, size_t numel);

    void append_vector(std::vector<double> &vec, const std::vector<double> &tail);

    std::vector<double> subvector_reverse(const std::vector<double> &vec, int idx_end, int idx_start);

    inline int max_val(const std::vector<int>& vec)
    {
        return std::max_element(vec.begin(), vec.end())[0];
    }

    void filter(std::vector<double> B, std::vector<double> A, const std::vector<double> &X, std::vector<double> &Y, std::vector<double> &Zi);

};

#endif // FILTFILT_H
