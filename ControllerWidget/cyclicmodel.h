#ifndef CYCLICMODEL_H
#define CYCLICMODEL_H

#include <vector>
#include <algorithm>
#include <iostream>
#include <Eigen/Dense>

class CyclicModel
{
public:
    CyclicModel();

    void train(const std::vector<double> data);
    void updatePeriod(const double shift);
    double getPrediction(const double timeShift);

private:
    void recalculate();

    // params
};

#endif // CYCLICMODEL_H
