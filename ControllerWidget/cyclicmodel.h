#ifndef CYCLICMODEL_H
#define CYCLICMODEL_H

#include <vector>
#include <algorithm>
#include <iostream>
#include <memory>
#include <limits>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <QString>
#include <QDir>
#include <QFile>
#include <QDataStream>
#include <QTextStream>
#include <QDebug>
#include <QElapsedTimer>

#include "filtfilt.h"

#include "../icebot_definitions.h"

class CyclicModel
{
public:
    CyclicModel();
    ~CyclicModel();

    // void operator = (const CyclicModel &Other);

    // add training data
    void addTrainingObservation(const EigenAffineTransform3d &T_BB_CT_curTipPos,
                        const EigenAffineTransform3d &T_BB_targetPos,
                        const EigenAffineTransform3d &T_Box_BBmobile,
                        const EigenAffineTransform3d &T_BB_Box,
                        const EigenAffineTransform3d &T_Bird4,
                        const double sampleTime);
    // reset model
    void resetModel();

    // get initial model
    void trainModel();

    // add respiration data
    void addObservation(const EigenAffineTransform3d &T_Bird4,
                        const double sampleTime);

    // update the period
    void updatePeriod(const double shift);

    // get a prediction
    double getPrediction(const double timeShift,
                         const EigenVectorPolar &x_polar,
                         const EigenVectorRectangular &x_rect);

    // Accessors
    const size_t getNumSamples() { return m_numSamples; }
    const bool isTrained() { return m_isTrained; }

    void setInVivo(const bool isInVivo);

    // testing function - external data load and save
    void loadData(QString filename, std::vector<double> &X);
    void load4x4Data(QString filename, EigenStdVecVector7d &X);
    void load4x4Data(QString filename, std::vector<std::vector<double>> &X);
    void saveFilteredData(QString filename, const std::vector<double> &Y);
    void saveFilteredData(QString filename, const EigenVectorFiltered &Y);
    void save4x4FilteredData(QString filename, const EigenStdVecVector7d &Y);
    void save4x4FilteredData(QString filename, const EigenMatrixFiltered &Y);

    void testLPF();

private:
    void retrainModel();

    void filterTrainingData();
    void filterNewObservations();

    double peakDetector(const bool runForInit);

    void cycle_recalculate(const std::vector<double> &inputs);

    // data members

    // Low Pass Filter
    filtfilt m_LowPassFilter;

    // in order to use deque with Eigen Transform, we need the typedef above
//    EigenDequeAffineTform3d m_BBfixed_CT,    // this stores all of the incoming CT points
//                            m_BBfixed_Instr, // this stores all of the incoming instr_x points
//                            m_BBfixed_BB,    // this stores all of the incoming BB points
//                            m_Bird4;         // this stores all of the incoming 4th EM sensor points

    // UNFILTERED DATA
    // x,y,z,xaxis,yaxis,zaxis,angle - only needed for training
    // TODO : how does replacing EigenStdVecVector7d with EigenMatrixFiltered affect performance?
    EigenStdVecVector7d m_BBfixed_CT,   // this stores all of the incoming CT points
                       m_BBfixed_Instr, // this stores all of the incoming instr_x points
                       m_BBfixed_BB;    // this stores all of the incoming BB points
//    std::vector<std::vector<double>> m_BBfixed_CT,   // this stores all of the incoming CT points
//                       m_BBfixed_Instr, // this stores all of the incoming instr_x points
//                       m_BBfixed_BB;    // this stores all of the incoming BB points
    // for the chest tracker, we only the need the x (benchtop) or -z (in vivo) axis of this
    std::vector<double> m_Bird4,     // this remains constant after initialization
                        m_Bird4_new; // most recent chest tracker data

    // FILTERED DATA
    // 7 components: x,y,z,xaxis,yaxis,zaxis,angle
    EigenMatrixFiltered m_BBfixed_CT_filtered, // this remains constant after initialization
                       m_BBfixed_BB_filtered;  // this remains constant after initialization
    // for the chest tracker, we only the need the x (benchtop) or -z (in vivo) axis of this
    EigenVectorFiltered m_Bird4_filtered,      // this remains constant after initialization
                        m_Bird4_filtered_new;  // most recent filtered chest tracker data

    // ***CURRENT*** RECTANGULAR COMPONENTS
    EigenMatrixRectangular m_BBfixed_CT_rectangular, // 7 components: x,y,z,xaxis,yaxis,zaxis,angle
                           m_BBfixed_BB_rectangular; // 7 components: x,y,z,xaxis,yaxis,zaxis,angle
    EigenVectorRectangular m_Bird4_rectangular;      // 1 component : x (benchtop) or -z (in vivo)

    // ***CURRENT*** POLAR COMPONENTS
    EigenMatrixPolar m_BBfixed_CT_polar, // 7 components: x,y,z,xaxis,yaxis,zaxis,angle
                     m_BBfixed_BB_polar; // 7 components: x,y,z,xaxis,yaxis,zaxis,angle
    EigenVectorPolar m_Bird4_polar;      // 1 component : x (benchtop) or -z (in vivo)

    // TIME DATA
    std::vector<double> m_timeData_init, // stores the time vector for the model initialization observations
                        m_timeData_new;  // stores time for the most recent observations

    double m_omega0; // frequency

    size_t m_numSamples; // number of observations added IN TOTAL

    bool m_isTrained; // is the model trained
    double m_lastTrainingTimestamp; // when last training was performed
    bool m_isInVivo; // are we in IN VIVO mode

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif // CYCLICMODEL_H
