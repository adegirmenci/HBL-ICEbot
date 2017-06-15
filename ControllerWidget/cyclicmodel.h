#ifndef CYCLICMODEL_H
#define CYCLICMODEL_H

#include <QObject>
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
#include <QtConcurrent/QtConcurrentRun>
#include <QFuture>
#include <QDateTime>

#include "filtfilt.h"

#include "../icebot_definitions.h"

class CyclicModel : public QObject
{
    Q_OBJECT
public:
    explicit CyclicModel(QObject *parent = 0);
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
    void updatePeriod(const double period);
    void updateNfutureSamples(int n) { m_nFutureSamples = n; }

    // Accessors
    const size_t getNumSamples() { return m_numSamples; }
    const bool isTrained() { return m_isTrained; }
    const bool isInVivoMode() { return m_isInVivo; }
    const double getOmega() { return m_omega0; }

    void setInVivo(const bool isInVivo);

    EigenVectorFiltered get_Bird4_filtered() { return m_Bird4_filtered; }
    EigenVectorFiltered get_Bird4_filtered_new() { return m_Bird4_filtered_new; }
    EigenVectorFiltered get_breathSignalFromModel() { return m_breathSignalFromModel; }

    EigenVector7d getT_BBfixed_CT_des() { return m_BBfixed_CT_des; }
    EigenVector7d getT_BBfixed_CTtraj_future_des() { return m_BBfixed_CTtraj_future_des; }
    EigenVector7d getT_BBfixed_BB_des() { return m_BBfixed_BB_des; }

    // testing function - external data load and save
    void loadData(QString filename, std::vector<double> &X);
    void load4x4Data(QString filename, EigenStdVecVector7d &X);
    void load4x4Data(QString filename, std::vector<std::vector<double>> &X);
    void saveFilteredData(QString filename, const std::vector<double> &Y);
    void saveFilteredData(QString filename, const EigenVectorFiltered &Y);
    void save4x4FilteredData(QString filename, const EigenStdVecVector7d &Y);
    void save4x4FilteredData(QString filename, const EigenMatrixFiltered &Y);

    void testLPF();

public slots:
    void setPlotFocus(int idx);

signals:
    void sendToPlotBird4(unsigned int plotID, double time, double val);

private:
    void retrainModel();

    void filterTrainingData();
    void filterNewObservations();

    double peakDetector(const bool runForInit);
    void peakDetectorForBreathModel();

    double getPrediction(const double timeShift,
                         const EigenVectorPolar &x_polar,
                         const EigenVectorRectangular &x_rect);
    void getPrediction7Axis(const double timeShift,
                            const EigenMatrixPolar &x_polar,
                            const EigenMatrixRectangular &x_rect,
                            EigenVector7d &X_des,
                            const double phase);

    void cycle_recalculate(const EigenMatrixFiltered &z_init,
                           EigenMatrixRectangular &x_rect,
                           EigenMatrixPolar &x_polar, const double omega0);
    void cycle_recalculate(const EigenVectorFiltered &z_init,
                           EigenVectorRectangular &x_rect,
                           EigenVectorPolar &x_polar, const double omega0);

    Eigen::MatrixXd cycle_recalculate_concurrentM(const EigenMatrixFiltered &z_init, const double omega0, const std::vector<double> &timeData);
    Eigen::VectorXd cycle_recalculate_concurrentV(const EigenVectorFiltered &z_init, const double omega0, const std::vector<double> &timeData);
    // data members

    // Low Pass Filter
    filtfilt m_LowPassFilter;

    // UNFILTERED DATA
    // x,y,z,xaxis,yaxis,zaxis,angle - only needed for training
    // TODO : how does replacing EigenStdVecVector7d with EigenMatrixFiltered affect performance?
    EigenStdVecVector7d m_BBfixed_CT,   // this stores all of the incoming CT points
                       m_BBfixed_Instr, // this stores all of the incoming instr_x points
                       m_BBfixed_BB;    // this stores all of the incoming BB points
    // for the chest tracker, we only the need the x (benchtop) or -z (in vivo) axis of this
    std::vector<double> m_Bird4,     // this remains constant after initialization
                        m_Bird4_new; // most recent chest tracker data

    // FILTERED DATA
    // 7 components: x,y,z,xaxis,yaxis,zaxis,angle
    EigenMatrixFiltered m_BBfixed_CT_filtered, // this remains constant after initialization
                       m_BBfixed_BB_filtered;  // this remains constant after initialization
    // for the chest tracker, we only the need the x (benchtop) or -z (in vivo) axis of this
    EigenVectorFiltered m_Bird4_filtered,      // this remains constant after initialization
                        m_Bird4_filtered_new,  // most recent filtered chest tracker data
                        m_breathSignalFromModel;

    // ***CURRENT*** RECTANGULAR COMPONENTS
    EigenMatrixRectangular m_BBfixed_CT_rectangular, // 7 components: x,y,z,xaxis,yaxis,zaxis,angle
                           m_BBfixed_BB_rectangular; // 7 components: x,y,z,xaxis,yaxis,zaxis,angle
    EigenVectorRectangular m_Bird4_rectangular;      // 1 component : x (benchtop) or -z (in vivo)

    // ***CURRENT*** POLAR COMPONENTS
    EigenMatrixPolar m_BBfixed_CT_polar, // 7 components: x,y,z,xaxis,yaxis,zaxis,angle
                     m_BBfixed_BB_polar; // 7 components: x,y,z,xaxis,yaxis,zaxis,angle
    EigenVectorPolar m_Bird4_polar;      // 1 component : x (benchtop) or -z (in vivo)

    // MODEL-BASED ESTIMATES OF CURRENT AND FUTURE POSITIONS
    EigenVector7d m_BBfixed_CT_des,
                  m_BBfixed_CTtraj_future_des,
                  m_BBfixed_BB_des;

    // TIME DATA
    std::vector<double> m_timeData_init, // stores the time vector for the model initialization observations
                        m_timeData_new;  // stores time for the most recent observations

    // PEAK DATA - MEAN OF THE LEFT AND RIGHT
    std::vector<double> m_respPeakMean, m_breathSignalPeakMean;

    size_t m_nFutureSamples; // how much into the future should we look?

    double m_omega0; // frequency
    double m_omega0_init;
    std::vector<double> m_periods;

    size_t m_numSamples; // number of observations added IN TOTAL

    bool m_isTrained; // is the model trained
    qint64 m_lastTrainingTimestamp; // when last training was performed
    bool m_isInVivo; // are we in IN VIVO mode

    int m_plotFocus;

    // concurrent execution
    QFuture<Eigen::MatrixXd> mConcurrent1, mConcurrent2;
    QFuture<Eigen::VectorXd> mConcurrent3;
    Eigen::MatrixXd m_BBfixed_CT_polarRect;
    Eigen::MatrixXd m_BBfixed_BB_polarRect;
    Eigen::VectorXd m_Bird4_polarRect;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif // CYCLICMODEL_H
