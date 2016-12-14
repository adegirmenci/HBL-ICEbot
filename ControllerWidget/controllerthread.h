#ifndef CONTROLLERTHREAD_H
#define CONTROLLERTHREAD_H

#include <QObject>
#include <QMutex>
#include <QMutexLocker>
#include <QThread>
#include <QString>
#include <QTime>
#include <QTimer>
#include <QElapsedTimer>
#include <QFile>
#include <QDebug>
#include <QSharedPointer>
#include <QAtomicInt>
#include <atomic>

#include <vector>
#include <memory>

// included in kinematics_4dof.h
//#include <Eigen/Dense>
//#include <Eigen/SVD>
//#include <Eigen/StdVector>
//#include <Eigen/Geometry>

#include <iostream>
//#include <fstream>
//#include <iterator>

#include "kinematics_4dof.h"
#include "filtfilt.h"
#include "cyclicmodel.h"

#include "../icebot_definitions.h"
#include "../AscensionWidget/3DGAPI/ATC3DG.h"
#include "gainswidget.h"

struct ModeFlags
{
    int coordFrame;
    int tethered;
    int instTrackState;
    int instTrackMode;
    int EKFstate;
    int inVivoMode;

    explicit ModeFlags(int cF = COORD_FRAME_WORLD,
                       int tth = MODE_TETHETERED,
                       int iTS = INST_TRACK_OFF,
                       int iTM = INST_TRACK_POSITION,
                       int EKFs = EKF_OFF,
                       int iVM = IN_VIVO_OFF) :
        coordFrame(cF), tethered(tth), instTrackState(iTS), instTrackMode(iTM), EKFstate(EKFs), inVivoMode(iVM)
    { }
};

Q_DECLARE_METATYPE(ModeFlags)
Q_DECLARE_METATYPE(EigenVectorFiltered)

class ControllerThread : public QObject
{
    Q_OBJECT
public:
    explicit ControllerThread(QObject *parent = 0);
    ~ControllerThread();

signals:
    void statusChanged(int event);
    void logData(QTime timeStamp,
                 int loopIdx,
                 int dataType,
                 std::vector<double> data);
    void logEvent(int source, // LOG_SOURCE
                  int logType, // LOG_TYPES
                  QTime timeStamp,
                  int eventID); // FRMGRAB_EVENT_IDS
    void logEventWithMessage(int source, // LOG_SOURCE
                  int logType, // LOG_TYPES
                  QTime timeStamp,
                  int eventID, // FRMGRAB_EVENT_IDS
                  QString &message);
    void sendMsgToWidget(QString msg);
    void logError(int source, // LOG_SOURCE
                  int logType, // LOG_TYPES
                  QTime timeStamp,
                  int errCode, // FRMGRAB_ERROR_CODES
                  QString message);
    void finished(); // emit upon termination
    void setEPOSservoTargetPos(std::vector<long> targetPos, bool moveAbsOrRel);

    void sendDataToRespModelWidget(int numSamples,
                                   bool isTrained,
                                   bool inVivoMode,
                                   double omega0);

public slots:
    void setEpoch(const QDateTime &epoch);
    void printThreadID();
    void receiveEMdata(QTime timeStamp,
                      int sensorID,
                      DOUBLE_POSITION_MATRIX_TIME_Q_RECORD data);
    void receiveLatestEMreading(std::vector<DOUBLE_POSITION_MATRIX_TIME_Q_RECORD> readings);
    void updateJointSpaceCommand(double pitch, double yaw, double roll, double trans);
    void updateConfigSpaceCommand(double alpha, double theta, double gamma, double d);
    void updateTaskSpaceCommand(double x, double y, double z, double delPsi, bool isAbsolute);

    void resetBB();

    void startControlCycle(); // start timer
    void stopControlCycle(); // stop timer

    const bool isControlling() { return m_keepControlling; }

    void setGains(GainsPYRT gains);
    void setLimits(ConvergenceLimits limits);
    void setModeFlags(ModeFlags flags);
    void setUSangle(double usAngle);

    void initializeRespModel();
    void re_initializeRespModel();
    void updateFutureSamples(int n);

private slots:
    void controlCycle(); // on a timer

private:
    // Instead of using "m_mutex.lock()"
    // use "QMutexLocker locker(&m_mutex);"
    // this will unlock the mutex when the locker goes out of scope
    mutable QMutex *m_mutex;

    // Timer for calling controlLoop every xxx msecs
    QTimer *m_timer;

    // Epoch for time stamps
    // During InitializeController(), check 'isEpochSet' flag
    // If Epoch is set externally from MainWindow, the flag will be true
    // Otherwise, Epoch will be set internally
    QDateTime m_epoch;
    bool m_isEpochSet;

    // Flag to indicate if Controller is ready
    // True if InitializeController was successful
    bool m_isReady;

    // Flag to tell that we are still controlling
    bool m_keepControlling;
    // Flag to abort actions
    bool m_abort;

    const int m_prec = 4; // precision for print operations

    // latest reading
    std::vector<DOUBLE_POSITION_MATRIX_TIME_Q_RECORD> m_prevReading;
    std::vector<DOUBLE_POSITION_MATRIX_TIME_Q_RECORD> m_latestReading;

    // transforms
    Eigen::Transform<double,3,Eigen::Affine> m_BB_Box,
                                             m_Box_SBm,
                                             m_STm_BT,
                                             m_BT_CT,
                                             m_BB_CT_curTipPos,
                                             m_BBfixed_CTorig,
                                             m_BB_SBm,
                                             m_BBfixed_BBmobile,
                                             m_BBmobile_BT,
                                             m_BBmobile_CT,
                                             m_curTipPos,
                                             m_ISm_INSTR,
                                             m_basTipPos_fixed,
                                             m_basTipPos_mobile,
                                             m_Box_BBfixed,
                                             m_Box_BBmobile,
                                             m_targetPos,
                                             m_BB_targetPos,
                                             m_currChest;

    // delta x,y,z,psi to target
    Eigen::Vector3d m_input_AbsXYZ, m_input_RelXYZ;
    double m_input_delPsi;
    Eigen::Vector4d m_dXYZPsi, m_deltaXYZPsiToTarget;
    double m_currGamma;

    // Respiration Model
    CyclicModel m_respModel;
    bool m_respModelInitializing;

    // gains
    GainsPYRT m_gains;
    // convergence limits
    ConvergenceLimits m_convLimits;
    // Mode Flags
    ModeFlags m_modeFlags;
    // US angle
    double m_USangle;

    // keep track of number of control cycles
    // an atomic variable alleviates the need to use mutexes during mutation
    std::atomic<unsigned int> m_numCycles;

    // Kinematics oject
    Kinematics_4DOF m_cathKin;

    // --- Private methods ---

    void computeCoordFrameWorld();  // Coord Frame: world
    void computeCoordFrameMobile(); // Coord Frame: mobile
    double computeSweep(const Eigen::Transform<double,3,Eigen::Affine> &currT, const Eigen::Vector3d &objXYZ);
    void updateGains();             // Update gains


    Eigen::Transform<double,3,Eigen::Affine> readTransformFromTxtFile(const QString &path);
    void loadConstants();

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// ----------------
// HELPER FUNCTIONS
// ----------------

inline const QString getCurrTimeStr();
inline const QString getCurrDateTimeStr();
static void Transform_From_EMreading(DOUBLE_POSITION_MATRIX_TIME_Q_RECORD &input, Eigen::Transform<double,3,Eigen::Affine> &output);

#endif // CONTROLLERTHREAD_H
