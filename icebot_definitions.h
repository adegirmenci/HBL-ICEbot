#ifndef ICEBOT_DEFINITIONS
#define ICEBOT_DEFINITIONS

//*****************************************************************************//
//
//	Author: Alperen Degirmenci
//
//	Date: 11/29/2015
//	Last Update: 09/07/2016
//
//	COPYRIGHT:		COPYRIGHT HARVARD BIOROBOTICS LABORATORY - 2015, 2016
//
//*****************************************************************************//
//
//	DESCRIPTION:   icebot_definitions.h
//
//	Header file containing definitions of constants and structures required.
//
//*****************************************************************************//

#include "../AscensionWidget/3DGAPI/ATC3DG.h"

// Alternative to using Boost for value of Pi
//const long double PI = 3.141592653589793238L;
//const double PI = 3.141592653589793;
//const float PI = 3.1415927;
enum LOG_TYPES
{
    LOG_INFO = 0, // regular messages
    LOG_WARNING, // warnings
    LOG_ERROR, // errors
    LOG_FATAL // fatal errors
};

enum LOG_SOURCE
{
    SRC_EM = 0,
    SRC_EPOS,
    SRC_FRMGRAB,
    SRC_EPIPHAN,
    SRC_LABJACK,
    SRC_OMNI,
    SRC_DATALOGGER,
    SRC_CONTROLLER,
    SRC_GUI,
    SRC_UNKNOWN
};

// **********************
// ********  EM  ********
// **********************

enum EM_ERROR_CODES
{
    EM_SUCCESS = 0,     //
    EM_FAIL,
    EM_BELOW_MIN_SAMPLE_RATE,
    EM_ABOVE_MAX_SAMPLE_RATE,
    EM_CANT_MUTATE_WHILE_RUNNING,
    EM_FREQ_SET_FAILURE,
    EM_SET_DATA_FORMAT_TYPE_FAILURE
};

enum EM_EVENT_IDS
{
	EM_INITIALIZE_BEGIN = 0,
	EM_INITIALIZE_FAILED,
	EM_INITIALIZED,
	EM_SENSORS_DETECTED,
	EM_TRANSMITTER_SET,
	EM_FREQ_DETECTED,
    EM_FREQ_SET,
    EM_FREQ_SET_FAILED,
	EM_ACQUISITION_STARTED,
	EM_START_ACQUISITION_FAILED,
	EM_ACQUIRE_FAILED,
	EM_ACQUISITION_STOPPED,
	EM_STOP_ACQUISITION_FAILED,
	EM_DISCONNECT_FAILED,
    EM_DISCONNECTED,
	EM_EPOCH_SET,
	EM_EPOCH_SET_FAILED,
	EM_SET_DATA_FORMAT_TYPE,
	EM_SET_DATA_FORMAT_TYPE_FAILED
};

enum EM_SENSOR_IDS
{
    EM_SENSOR_BB = 0,
    EM_SENSOR_BT,
    EM_SENSOR_INST,
    EM_SENSOR_CHEST
};

static const char* const EM_SENSOR_NAMES[] = {"BB", "BT", "INST", "CHEST"};

static const int EM_DEFAULT_SAMPLE_RATE = 150;
static const int EM_MIN_SAMPLE_RATE = 20;
static const int EM_MAX_SAMPLE_RATE = 250;

#define EM_DELTA_T_SIZE 151

//typedef DOUBLE_POSITION_QUATERNION_TIME_Q_RECORD EM_RECORD_TYPE;
// typedef DOUBLE_POSITION_MATRIX_TIME_Q_RECORD EM_RECORD_TYPE;

// Preferred EM reading type is : DOUBLE_POSITION_QUATERNION_TIME_Q_RECORD

// **********************
// ******** EPOS ********
// **********************

enum EPOS_ERROR_CODES
{
    EPOS_SUCCESS = 0,
    EPOS_FAIL,
    EPOS_FAILED_TO_CONNECT
};

enum EPOS_EVENT_IDS
{
    EPOS_INITIALIZE_BEGIN = 0,
    EPOS_INITIALIZE_FAILED,
    EPOS_INITIALIZED,
    EPOS_SERVO_LOOP_STARTED,
    EPOS_SERVO_LOOP_STOPPED,
    EPOS_SERVO_TO_POS_FAILED,
    EPOS_EPOCH_SET,
    EPOS_EPOCH_SET_FAILED,
    EPOS_DISABLE_MOTOR_FAILED,
    EPOS_DISABLE_MOTOR_SUCCESS,
    EPOS_DEVICE_CANT_CONNECT,
    EPOS_UPDATE_QC_FAILED,
    EPOS_HALT_FAILED,
    EPOS_DISCONNECTED,
    EPOS_DISCONNECT_FAILED
};

enum EPOS_DATA_IDS
{
    EPOS_COMMANDED = 0,
    EPOS_READ
};

enum EPOS_MOTOR_STATUS
{
    EPOS_MOTOR_DISABLED = 0,
    EPOS_MOTOR_ENABLED,
    EPOS_MOTOR_FAULT,
    EPOS_MOTOR_CANT_CONNECT,
    EPOS_INVALID_MOTOR_ID
};

// enum EPOS_MOTOR_IDS
// {
//     TRANS_MOTOR_ID = 1,
//     PITCH_MOTOR_ID,
//     YAW_MOTOR_ID,
//     ROLL_MOTOR_ID
// };

static const int EPOS_NUM_MOTORS = 4;

static const int TRANS_MOTOR_ID = 1;
static const int PITCH_MOTOR_ID = 2;
static const int YAW_MOTOR_ID = 3;
static const int ROLL_MOTOR_ID = 4;

static const int EPOS_MOTOR_IDS[EPOS_NUM_MOTORS] = {TRANS_MOTOR_ID, PITCH_MOTOR_ID, YAW_MOTOR_ID, ROLL_MOTOR_ID};

static const int TRANS_AXIS_ID = 0;
static const int PITCH_AXIS_ID = 1;
static const int YAW_AXIS_ID = 2;
static const int ROLL_AXIS_ID = 3;

static const int EPOS_AXIS_IDS[EPOS_NUM_MOTORS] = {TRANS_AXIS_ID, PITCH_AXIS_ID, YAW_AXIS_ID, ROLL_AXIS_ID};

// original values from Alienware
//static const int EPOS_VELOCITY[EPOS_NUM_MOTORS] = {5000, 5000, 5000, 5000};
//static const int EPOS_ACCEL[EPOS_NUM_MOTORS] = {20000, 50000, 50000, 50000};
//static const int EPOS_DECEL[EPOS_NUM_MOTORS] = {20000, 30000, 30000, 30000};

// better values
static const int EPOS_VELOCITY[EPOS_NUM_MOTORS] = {5000, 10000, 10000, 10000};
static const int EPOS_ACCEL[EPOS_NUM_MOTORS] = {50000, 80000, 80000, 80000};
static const int EPOS_DECEL[EPOS_NUM_MOTORS] = {50000, 70000, 70000, 70000};

// values for mapping
//static const int EPOS_VELOCITY[EPOS_NUM_MOTORS] = {5000, 5000, 5000, 5000};
//static const int EPOS_ACCEL[EPOS_NUM_MOTORS] = {50000, 30000, 30000, 30000};
//static const int EPOS_DECEL[EPOS_NUM_MOTORS] = {50000, 20000, 20000, 20000};

static const long EPOS_TRANS_MIN = -320000;
static const long EPOS_TRANS_MAX = 320000;
static const long EPOS_PITCH_MIN = -45000;
static const long EPOS_PITCH_MAX = 45000;
static const long EPOS_YAW_MIN = -45000;
static const long EPOS_YAW_MAX = 45000;
static const long EPOS_ROLL_MIN = -1381000;
static const long EPOS_ROLL_MAX = 1381000;

static const int EPOS_MOTOR_LIMITS[EPOS_NUM_MOTORS][2] =
                                           { {EPOS_TRANS_MIN, EPOS_TRANS_MAX},
                                             {EPOS_PITCH_MIN, EPOS_PITCH_MAX},
                                             {EPOS_YAW_MIN, EPOS_YAW_MAX},
                                             {EPOS_ROLL_MIN, EPOS_ROLL_MAX}   };

static const long EPOS_TRANS_RAD2QC = -8503937;
static const long EPOS_PITCH_RAD2QC = 45837;
static const long EPOS_YAW_RAD2QC = -45837;
static const long EPOS_ROLL_RAD2QC = -45837;

// ***********************
// ******  LabJack  ******
// ***********************

// enum LABJACK_ERROR_CODES ---- use LJ_ERROR codes instead from LabJackUD.h

enum LABJACK_EVENT_IDS
{
    LABJACK_CONNECT_BEGIN = 0,
    LABJACK_CONNECT_FAILED,
    LABJACK_CONNECTED,
    LABJACK_INITIALIZE_BEGIN,
    LABJACK_INITIALIZE_FAILED,
    LABJACK_INITIALIZED,
    LABJACK_LOOP_STARTED,
    LABJACK_LOOP_STOPPED,
    LABJACK_EPOCH_SET,
    LABJACK_EPOCH_SET_FAILED,
    LABJACK_DISCONNECTED,
    LABJACK_DISCONNECT_FAILED
};

// ***********************
// ******  FrmGrab  ******
// ***********************

enum FRMGRAB_ERROR_CODES
{
    FRMGRAB_SUCCESS = 0,     //
    FRMGRAB_FAIL,
    FRMGRAB_CANT_MUTATE_WHILE_RUNNING
};

enum FRMGRAB_EVENT_IDS
{
    FRMGRAB_CONNECT_BEGIN = 0,
    FRMGRAB_CONNECT_FAILED,
    FRMGRAB_CONNECTED,
    FRMGRAB_INITIALIZE_BEGIN,
    FRMGRAB_INITIALIZE_FAILED,
    FRMGRAB_INITIALIZED,
    FRMGRAB_LOOP_STARTED,
    FRMGRAB_LOOP_STOPPED,
    FRMGRAB_LIVE_FEED_STARTED,
    FRMGRAB_LIVE_FEED_STOPPED,
    FRMGRAB_EPOCH_SET,
    FRMGRAB_EPOCH_SET_FAILED,
    FRMGRAB_DISCONNECTED,
    FRMGRAB_DISCONNECT_FAILED
};

static const int FRMGRAB_IM_WIDTH = 1920; // 640;
static const int FRMGRAB_IM_HEIGHT = 1080; // 480;
static const int FRMGRAB_FPS = 60;

// **************************
// ******  CONTROLLER  ******
// **************************

#define PERIOD_FILTER_SIZE 150

static const double PITCH_WP = 7.3896;
static const double PITCH_WY = 1.2693;
static const double PITCH_BIAS = 0.2529;
static const double PITCH_SUM = 8.9118;
static const double YAW_WY = 5.9987;
static const double YAW_WP = 0.3187;
static const double YAW_BIAS = -0.1743;
static const double YAW_SUM = 6.1431;

enum CONTROLLER_ERROR_CODES
{
    CONTROLLER_SUCCESS = 0,     //
    CONTROLLER_FAIL,
    CONTROLLER_CANT_MUTATE_WHILE_RUNNING
};

enum CONTROLLER_EVENT_IDS
{
    CONTROLLER_INITIALIZE_BEGIN = 0,
    CONTROLLER_INITIALIZE_FAILED,
    CONTROLLER_INITIALIZED,
    CONTROLLER_LOOP_STARTED,
    CONTROLLER_LOOP_STOPPED,
    CONTROLLER_RESP_MODEL_INIT_BEGIN,
    CONTROLLER_RESP_MODEL_INITIALIZED,
    CONTROLLER_RESP_MODEL_INIT_FAILED,
    CONTROLLER_RESP_MODEL_STOPPED,
    CONTROLLER_RESETBB_SUCCESS,
    CONTROLLER_EPOCH_SET,
    CONTROLLER_EPOCH_SET_FAILED,
    CONTROLLER_RESET,
    CONTROLLER_RESET_FAILED,
    CONTROLLER_SWEEP_STARTED,
    CONTROLLER_SWEEP_ABORTED
};

enum CONTROLLER_DATA_IDS
{
    CONTROLLER_DXYZPSI = 0, // 4 values
    CONTROLLER_USER_XYZDXYZPSI, // 7 values
    CONTROLLER_CURR_PSY_GAMMA, // 2 values
    CONTROLLER_T_BB_CT_curTipPos, // 16 values
    CONTROLLER_CURR_TASK, // 4 values
    CONTROLLER_PERIOD, // 1 value
    CONTROLLER_BIRD4_MODEL_PARAMS, // 19 = 10 polar + 9 rect
    CONTROLLER_RESETBB, // 16 values
    CONTROLLER_MODES, // 6 values
    CONTROLLER_USANGLE, // 1 value
    CONTROLLER_SWEEP_START, // 4 values
    CONTROLLER_NEXT_SWEEP,
    CONTROLLER_SWEEP_CONVERGED
};

enum CONTROLLER_SPACES
{
    JOINT_SPACE = 0,
    CONFIG_SPACE,
    TASK_SPACE
};

enum CONTROLLER_SPACE_VARIABLES
{
    JOINT_SPACE_TRANS = 0,
    JOINT_SPACE_PITCH,
    JOINT_SPACE_YAW,
    JOINT_SPACE_ROLL,
    CONFIG_SPACE_ALPHA,
    CONFIG_SPACE_THETA,
    CONFIG_SPACE_GAMMA,
    CONFIG_SPACE_D,
    TASK_SPACE_X,
    TASK_SPACE_Y,
    TASK_SPACE_Z,
    TASK_SPACE_DEL_PSI
};

// Mode flags
enum COORD_FRAME_MODE
{
    COORD_FRAME_WORLD = 0,
    COORD_FRAME_MOBILE
};

enum TETHER_MODE
{
    MODE_TETHETERED = 0,
    MODE_RELATIVE
};

enum INST_TRACK_STATE
{
    INST_TRACK_OFF = 0,
    INST_TRACK_ON
};

enum INST_TRACK_MODE
{
    INST_TRACK_POSITION = 0,
    INST_TRACK_IMAGER
};

enum EKF_STATE
{
    EKF_OFF = 0,
    EKF_ON
};

enum IN_VIVO_MODE
{
    IN_VIVO_OFF = 0,
    IN_VIVO_ON
};

enum MODEL_PLOT_INDEX
{
    RESP_MODEL_PLOT_BIRD4 = 0,
    RESP_MODEL_PLOT_CT
};

enum SWEEP_MODES
{
    SWEEP_INACTIVE = 0,
    SWEEP_WAIT_TO_CONVERGE,
    SWEEP_CONVERGED,
    SWEEP_CONVERGED_ACQUIRING,
    SWEEP_NEXT,
    SWEEP_DONE
};

static const int CONTROLLER_LOOP_TIMER_MSEC = 1;

// *************************
// ******  FILTERING  ******
// *************************

#define FILTER_ORDER      50                        // LPF filter order
#define SAMPLE_DELTA_TIME 0.005992 //0.006667       // delta time b/w EM readings
#define HEART_RATE        100                       // animal heartrate
#define N_HARMONICS       4                         // number of Fourier decomp harmonics
#define N_STATES          N_HARMONICS*2 + 2         // number of states
#define N_RECT            N_STATES - 1              // NUM_STATES of Rectangular components
#define N_POLAR           N_STATES                  // NUM_STATES of Polar components
#define N_SAMPLES         2750                      // CYCLE_DATA_SIZE
#define EDGE_EFFECT       35                        // extent of edge effects
#define N_FILTERED        N_SAMPLES - 2*EDGE_EFFECT // Filtered data length
#define BREATH_RATE       5.0                       // respiration period (seconds)
#define PEAK_THRESHOLD    0.80                      // For peak detection

// ***********************
// ***** DATA LOGGER *****
// ***********************

static const int DATALOG_NUM_FILES = 8;
static const unsigned short DATALOG_EM_ID = 0;
static const unsigned short DATALOG_ECG_ID = 1;
static const unsigned short DATALOG_EPOS_ID = 2;
static const unsigned short DATALOG_FrmGrab_ID = 3;
static const unsigned short DATALOG_Log_ID = 4;
static const unsigned short DATALOG_Error_ID = 5;
static const unsigned short DATALOG_Note_ID = 6;
static const unsigned short DATALOG_Control_ID = 7;

static const unsigned short DATALOG_FILE_IDS[DATALOG_NUM_FILES] =
                                                  {DATALOG_EM_ID,
                                                   DATALOG_ECG_ID,
                                                   DATALOG_EPOS_ID,
                                                   DATALOG_FrmGrab_ID,
                                                   DATALOG_Log_ID,
                                                   DATALOG_Error_ID,
                                                   DATALOG_Note_ID,
                                                   DATALOG_Control_ID};

enum DATALOG_EVENT_IDS
{
    DATALOG_INITIALIZE_BEGIN = 0,
    DATALOG_INITIALIZE_FAILED,
    DATALOG_INITIALIZED,
    DATALOG_FILE_OPENED,
    DATALOG_FILE_CLOSED,
    DATALOG_FOLDER_ERROR,
    DATALOG_FILE_ERROR,
    DATALOG_FILE_DATA_LOGGED,
    DATALOG_EM_FILE_OPENED,
    DATALOG_EM_FILE_CLOSED,
    DATALOG_EM_FILE_DATA_LOGGED,
    DATALOG_ECG_FILE_OPENED,
    DATALOG_ECG_FILE_CLOSED,
    DATALOG_ECG_FILE_DATA_LOGGED,
    DATALOG_EPOS_FILE_OPENED,
    DATALOG_EPOS_FILE_CLOSED,
    DATALOG_EPOS_FILE_DATA_LOGGED,
    DATALOG_FRMGRAB_FILE_OPENED,
    DATALOG_FRMGRAB_FILE_CLOSED,
    DATALOG_FRMGRAB_FILE_DATA_LOGGED,
    DATALOG_LOG_FILE_OPENED,
    DATALOG_LOG_FILE_CLOSED,
    DATALOG_LOG_FILE_DATA_LOGGED,
    DATALOG_ERROR_FILE_OPENED,
    DATALOG_ERROR_FILE_CLOSED,
    DATALOG_ERROR_FILE_DATA_LOGGED,
    DATALOG_NOTE_FILE_OPENED,
    DATALOG_NOTE_FILE_CLOSED,
    DATALOG_NOTE_FILE_DATA_LOGGED,
    DATALOG_CONTROL_FILE_OPENED,
    DATALOG_CONTROL_FILE_CLOSED,
    DATALOG_CONTROL_FILE_DATA_LOGGED,
	DATALOG_EPOCH_SET,
    DATALOG_EPOCH_SET_FAILED,
    DATALOG_LOGGING_STARTED,
    DATALOG_LOGGING_STOPPED,
    DATALOG_CLOSED
};

// ************************
// ***** FRAME SERVER *****
// ************************

enum FRMSRVR_EVENT_IDS
{
    FRMSRVR_STARTED = 0,
    FRMSRVR_START_FAILED,
    FRMSRVR_CLOSED,
    FRMSRVR_CLOSE_FAILED,
    FRMSRVR_NEW_CONNECTION,
    FRMSRVR_SOCKET_NOT_READABLE,
    FRMSRVR_FRAME_RECEIVED,
    FRMSRVR_EPOCH_SET,
    FRMSRVR_EPOCH_SET_FAILED
};


// ************************
// ***** FRAME CLIENT *****
// ************************

enum FRMCLNT_EVENT_IDS
{
    FRMCLNT_CONNECTED = 0,
    FRMCLNT_CONNECTION_FAILED,
    FRMCLNT_DISCONNECTED,
    FRMCLNT_DISCONNECTION_FAILED,
    FRMCLNT_SOCKET_NOT_WRITABLE,
    FRMCLNT_FRAME_SENT,
    FRMCLNT_EPOCH_SET,
    FRMCLNT_EPOCH_SET_FAILED,
    FRMCLNT_FIRST_FRAME_NOT_RECEIVED
};

// *************************
// ***** VOLUME SERVER *****
// *************************

enum VOLSRVR_EVENT_IDS
{
    VOLSRVR_STARTED = 0,
    VOLSRVR_START_FAILED,
    VOLSRVR_CLOSED,
    VOLSRVR_CLOSE_FAILED,
    VOLSRVR_NEW_CONNECTION,
    VOLSRVR_SOCKET_NOT_READABLE,
    VOLSRVR_VOLUME_RECEIVED,
    VOLSRVR_EPOCH_SET,
    VOLSRVR_EPOCH_SET_FAILED
};

// *************************
// ***** VOLUME CLIENT *****
// *************************

enum VOLCLNT_EVENT_IDS
{
    VOLCLNT_CONNECTED = 0,
    VOLCLNT_CONNECTION_FAILED,
    VOLCLNT_DISCONNECTED,
    VOLCLNT_DISCONNECTION_FAILED,
    VOLCLNT_SOCKET_NOT_WRITABLE,
    VOLCLNT_VOLUME_SENT,
    VOLCLNT_EPOCH_SET,
    VOLCLNT_EPOCH_SET_FAILED,
    VOLCLNT_NOT_READY
};

#endif // ICEBOT_DEFINITIONS


// NEVER PASS EIGEN TYPES BY VALUE TO FUNCTIONS, ALWAYS PASS BY REFERENCE - BUT IT'S OK IF FUNCTIONS RETURN EIGEN TYPES BY VALUE
// IF USING VECTOR/LIST/DEQUE/ETC TO STORE EIGEN FIXED TYPES (LIKE TRANSFORM), USE THE EIGEN::ALIGNED_ALLOCATOR IN DECLARING THE CONTAINER TYPE
