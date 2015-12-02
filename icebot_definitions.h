#ifndef ICEBOT_DEFINITIONS
#define ICEBOT_DEFINITIONS

//*****************************************************************************//
//
//	Author: Alperen Degirmenci
//
//	Date: 11/29/2015
//
//	COPYRIGHT:		COPYRIGHT HARVARD BIOROBOTICS LABORATORY - 2015
//
//*****************************************************************************//
//
//	DESCRIPTION:   icebot_definitions.h
//
//	Header file containing definitions of constants and structures required.
//
//*****************************************************************************//



#endif // ICEBOT_DEFINITIONS

enum LOG_TYPES
{
    LOG_INFO, // regular messages
    LOG_WARNING, // warnings
    LOG_ERROR, // errors
    LOG_FATAL // fatal errors
};

enum EM_ERROR_CODES
{
    EM_SUCCESS = 0,     //
    EM_FAIL,
    EM_BELOW_MIN_SAMPLE_RATE,
    EM_ABOVE_MAX_SAMPLE_RATE,
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

#define EM_DEFAULT_SAMPLE_RATE 150
#define EM_MIN_SAMPLE_RATE 20
#define EM_MAX_SAMPLE_RATE 250

enum EPOS_ERROR_CODES
{
    EPOS_SUCCESS = 0,
    EPOS_FAILED_TO_CONNECT
};
