#ifndef CYCLICMODEL_H
#define CYCLICMODEL_H

#include <vector>
#include <deque>
#include <algorithm>
#include <iostream>
#include <memory>
#include <limits>
#include <Eigen/Dense>

#define N_HARMONICS 4              // NUM_HARM
#define N_STATES 10                // NUM_STATES
#define SAMPLE_DELTA_TIME 0.005992 // SAMPLE_TIME
#define N_SAMPLES 4000             // CYCLE_DATA_SIZE
#define EDGE_EFFECT 35             // EDGE_EFFECT
#define FILTER_ORDER 50            // FILTERORDER
#define BREATH_RATE 7.5            // BREATHRATE
#define PEAK_THRESHOLD 0.80        // For peak detection

typedef Eigen::Transform<double,3,Eigen::Affine> EigenAffineTransform3d;

class CyclicModel
{
public:
    CyclicModel();
    ~CyclicModel();

    void addObservation(const EigenAffineTransform3d &T_BB_CT_curTipPos,
                        const EigenAffineTransform3d &T_BB_targetPos,
                        const EigenAffineTransform3d &T_Box_BBmobile,
                        const EigenAffineTransform3d &T_BB_Box,
                        const EigenAffineTransform3d &T_Bird4,
                        const double sampleTime);
    void trainModel(const std::vector<double> data);
    void updatePeriod(const double shift);
    double getPrediction(const double timeShift,
                         const std::vector<double> &x_polar,
                         const std::vector<double> &x_rect);

    // Accessors
    const size_t getNumSamples() { return m_numSamples; }
    const bool isTrained() { return isTrained; }

private:
    void retrainModel();

    std::shared_ptr< std::vector<double> > cycle_recalculate(const std::vector<double> &inputs);

    // data members
    std::deque< EigenAffineTransform3d > m_BBfixed_CT,    // this stores all of the incoming CT points
                                         m_BBfixed_Instr, // this stores all of the incoming instr_x points
                                         m_BBfixed_BB,    // this stores all of the incoming BB points
                                         m_Bird4;         // this stores all of the incoming 4th EM sensor points

    std::deque<double> m_timeData, m_breathingSignal; // stores time, and data to transfer to period updater

    size_t m_numSamples;

    bool m_isTrained;
    double m_lastTrainingTimestamp;

};

#endif // CYCLICMODEL_H
