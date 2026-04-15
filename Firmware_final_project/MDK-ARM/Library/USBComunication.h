#ifndef __USB_COMMUNICATION_H__
#define __USB_COMMUNICATION_H__

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/
	
#include "stdint.h"	
#include "stdbool.h"

typedef enum
{
    CMD_SERVO_ON	   = 0x01,
    CMD_SERVO_OFF     = 0x02,
    CMD_STOP_PPF      = 0x03,
		CMD_APPLY_SPEED_FILTER = 0x04,
		CMD_APPLY_BODE_OPEN_CLOSE_LOOP = 0x05,
    CMD_APPLY_CTUNING = 0x06 ,
    CMD_APPLY_STUNING = 0x07,
    CMD_START_CTUNING = 0x08 ,
    CMD_STOP_CTUNING = 0x09,
    CMD_START_AUTOCOMMUTATION = 0x0A,
    CMD_START_TORQUECONTROL = 0x0C,
    CMD_STOP_TORQUECONTROL = 0x0D,
    CMD_START_SPEEDCONTROL = 0x0E,
    CMD_STOP_SPEEDCONTROL = 0x0F,
    CMD_START_POSITIONCONTROL = 0x10,
    CMD_STOP_POSITIONCONTROL = 0x11,
    CMD_SERVO_JOGPROGRAM = 0x12,
    CMD_SVOFF_JOGPROGRAM = 0x13,
    CMD_START_JOGPROGRAM = 0x14,
    CMD_STOP_JOGPROGRAM = 0x15,
    CMD_START_MJOG_INC = 0x16,
    CMD_START_MJOG_DEC = 0x17,
    CMD_STOP_MJOG = 0x18,
    CMD_RECALC_PROFILECONSTANT = 0x19,
    CMD_START_STUNING = 0x1A,
    CMD_STOP_STUNING = 0x1B,
		CMD_UPDATE_MONITOR = 0x1C,
		CMD_APPLY_TRACE = 0x1D,
		CMD_APPLY_MJOG = 0x1E,
		CMD_APPLY_JOGPROGRAM = 0x1F,
    CMD_START_PSTUNING = 0x20,
    CMD_STOP_PSTUNING = 0x21,
    CMD_EMIT_MOTORCHANGE = 0x22,
    CMD_SAVE_EEPROM = 0x23,
    CMD_START_TRACE = 0x24,
    CMD_READ_DRIVER = 0x25,
    CMD_READ_MOTOR = 0x26,
    CMD_WRITE_DRIVER = 0x27,
    CMD_WRITE_MOTOR = 0x28,
    CMD_ACK_FAULT = 0x29,
    CMD_STOP_TRACE = 0x2A,
    CMD_WRITE_TO_FLASH = 0x2B,
		CMD_APPLY_FILTER_NGUYEN = 0x2C,
		CMD_PLOT_BODE_NGUYEN = 0x2D,
		CMD_APPLY_ATUNING_THINH = 0x2E,
		CMD_EXECUTE_ATUNING_THINH = 0x2F,
		CMD_START_COMMUTATION_NEW = 0x50,
		CMD_START_AUTOTUNING_T = 0x51,
		CMD_STOP_AUTOTUNING_T = 0x52,
		CMD_UPDATE_TUNING_GAIN = 0x53,
		CMD_CONTINUE_AUTO_TUNING_STATE = 0x54,
		CMD_START_OPEN_LOOP_VF = 0x55,
		CMD_STOP_OPEN_LOOP_VF = 0x56,
		CMD_APPLY_ID_SQUARE_TUNING = 0x57,
		CMD_START_ID_SQUARE_TUNING = 0x58,
		CMD_STOP_ID_SQUARE_TUNING = 0x59,
		CMD_SET_CONTROL_TIMING_MODE = 0x5A,
		CMD_START_ENCODER_ALIGNMENT = 0x5B,
		CMD_START_FOC_DIRECTION_TEST = 0x5C,
		CMD_START_FOC_ANGLE_FIT = 0x5D,
}Command_e;

typedef enum
{
	CMD_CTUNNING_DATA = 0x30,
	CMD_STUNNING_DATA = 0x31,
	CMD_READ_DRIVER_DATA = 0x32,
	CMD_READ_MOTOR_DATA = 0x33,
	CMD_TRACE_DATA = 0x34,
	CMD_MONITOR_DATA = 0x35,
	CMD_FFT_DATA_NGUYEN = 0x36,
	CMD_FFT_DATA_THINH = 0x37,
	CMD_AUTOTUNING_DATA_THINH = 0x38,
	MTR_CODE_ERROR = 0xE1
}UpdateDataCmd_e;

typedef enum
{
	RUN_MODE_FOC = 0u,
	RUN_MODE_OPEN_LOOP_VF = 1u,
	RUN_MODE_ALIGNMENT_ONLY = 2u,
	RUN_MODE_AUTOTUNE = 3u
}RunMode_e;

typedef enum
{
	ID_SQUARE_TUNING_MODE_SQUARE_WAVE = 0u,
	ID_SQUARE_TUNING_MODE_ALIGNMENT_HOLD = 1u
}IdSquareTuningMode_e;

typedef enum
{
	ID_SQUARE_ANGLE_TEST_NONE = 0u,
	ID_SQUARE_ANGLE_TEST_PLUS_90 = 1u,
	ID_SQUARE_ANGLE_TEST_MINUS_90 = 2u,
	ID_SQUARE_ANGLE_TEST_PLUS_180 = 3u
}IdSquareAngleTest_e;

typedef enum
{
	ENCODER_ALIGNMENT_POLICY_POWER_ON = 0u,
	ENCODER_ALIGNMENT_POLICY_MANUAL_SAVE = 1u
}EncoderAlignmentPolicy_e;

typedef enum
{
	ENCODER_ALIGNMENT_STATUS_IDLE = 0u,
	ENCODER_ALIGNMENT_STATUS_REQUESTED = 1u,
	ENCODER_ALIGNMENT_STATUS_RUNNING = 2u,
	ENCODER_ALIGNMENT_STATUS_DONE = 3u,
	ENCODER_ALIGNMENT_STATUS_FAULT = 4u
}EncoderAlignmentStatus_e;

typedef enum
{
	FOC_DIRECTION_TEST_IDLE = 0u,
	FOC_DIRECTION_TEST_RUNNING = 1u,
	FOC_DIRECTION_TEST_DONE_OK = 2u,
	FOC_DIRECTION_TEST_DONE_FLIPPED = 3u,
	FOC_DIRECTION_TEST_INCONCLUSIVE = 4u,
	FOC_DIRECTION_TEST_FAULT = 5u
}FocDirectionTestStatus_e;

typedef enum
{
	CURRENT_CALIB_STATUS_IDLE = 0u,
	CURRENT_CALIB_STATUS_RUNNING = 1u,
	CURRENT_CALIB_STATUS_DONE = 2u,
	CURRENT_CALIB_STATUS_TIMEOUT = 3u,
	CURRENT_CALIB_STATUS_OFFSET_INVALID = 4u
}CurrentCalibStatus_e;

typedef struct
{
	uint8_t TransmitData[1024];
	uint8_t ReadMotionMonitorData;
	uint8_t ReadDriverParameter;
	uint8_t ReadMotorParameter;
	uint8_t ReadTuningData;
	uint8_t ReadTraceData;
	uint8_t ReadFFTData_th;
	uint8_t ReadFFTData_ng;
	uint8_t ReadAutoTuningData_T;
	uint8_t PriorityFlag;
	uint8_t TotalLength, Counter;
	bool bDataProcesable;
	bool SendError;
}USB_Comunication_t;

typedef struct
{
	float *Data1;
	float *Data2;
	float *Data3;
	float *Data4;
	uint16_t Counter;
	uint16_t u16Cnt2;
	uint8_t Enable;
	uint8_t Mode;
	uint8_t Finish;
	uint8_t u8MaxCntSample;
	uint8_t u8CntSample;
}TraceData;

typedef struct
{
	uint8_t Enable;
	uint8_t Mode;
	uint8_t AlignmentDone;
	uint8_t ElectricalAngleTestMode;
	uint8_t CurrentPolarityInvertTest;
	uint8_t CurrentUvSwapTest;
	uint16_t AlignmentCounter;
	float fAmplitudeCmd;
	float fFrequencyCmd;
	float fCurrentKpCmd;
	float fCurrentKiCmd;
	float fAmplitudeApplied;
	float fFrequencyApplied;
	float fPhase;
	float fVoltageLimitApplied;
	float fAlignmentCurrentApplied;
	int32_t OffsetCaptured;
}IdSquareTuning_t;

void UpdateDriverParameter(float *DriverParameter);

void UpdateMotorParameter(float *MotorParameter);

int16_t USB_TransmitData(USB_Comunication_t* USB_Comunicate);

void USB_ReceiveData(uint8_t *data, uint32_t SizeOfData);

void USB_ProcessData(USB_Comunication_t* USB_Comunicate);

void SendData(UpdateDataCmd_e uCommand, uint16_t uDataLength, uint8_t *uSetData);
	
#ifdef __cplusplus
}
#endif /*__cplusplus*/

#endif /*__Usb_Communication*/

