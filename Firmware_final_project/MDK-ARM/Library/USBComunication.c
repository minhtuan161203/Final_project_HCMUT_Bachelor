#include "USBComunication.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "Parameter.h"
#include "StateMachine.h"
#include "define.h"
#include "motor_autotune.h"
#include "pwm.h"
#include "usbd_cdc_if.h"

#define RX_BYTE_BUFFER_SIZE 512u
#define RX_FRAME_BUFFER_SIZE 260u
#define TX_FRAME_BUFFER_SIZE 256u
#define USB_COMM_OK 0u
#define USB_COMM_FAIL 1u
#define FOC_DEBUG_TEXT_BUFFER_SIZE 160u

static const uint8_t STX = 0x02u;
static const uint8_t ACK = 0xF0u;
static const uint8_t ACK_ERROR = 0xFFu;
static const uint8_t SYN = 0x16u;
static const uint8_t ETX = 0x03u;

static volatile uint16_t s_rx_head = 0u;
static volatile uint16_t s_rx_tail = 0u;
static uint8_t s_rx_buffer[RX_BYTE_BUFFER_SIZE];
static volatile uint8_t s_foc_debug_pending = 0u;
static uint8_t s_foc_debug_length = 0u;
static uint8_t s_foc_debug_buffer[FOC_DEBUG_TEXT_BUFFER_SIZE];

typedef struct
{
	uint8_t buffer[RX_FRAME_BUFFER_SIZE];
	uint16_t index;
	uint16_t expected_length;
	uint8_t receiving;
} UsbFrameParser_t;

static UsbFrameParser_t s_parser = { {0}, 0u, 0u, 0u };

extern Parameterhandle_t Parameter;
extern CurrentSensor_t Current_Sensor;
extern StateMachine_t StateMachine;
extern USB_Comunication_t USB_Comm;
extern float DriverParameter[DRIVER_PARAMETER_COUNT];
extern float MotorParameter[32];
extern float gIdRefA;
extern float gIqRefA;
extern volatile float gTargetSpeedRpm;
extern volatile float gTargetPositionCounts;
extern volatile float gCommandedSpeedRpm;
extern volatile uint8_t gRunMode;
extern volatile float gVfFrequencyHz;
extern volatile float gVfVoltageV;
extern volatile float gDebugSpeedRawRpm;
extern volatile float gDebugSpeedRawRpmAvg;
extern volatile float gDebugObservedElectricalHz;
extern volatile float gDebugObservedElectricalHzAvg;
extern volatile float gDebugOpenLoopElectricalHzCmd;
extern volatile float gDebugOpenLoopSyncRpmCmd;
extern volatile float gDebugExpectedDeltaPosSync;
extern volatile float gDebugDeltaPosAvg;
extern volatile float gDebugEncoderTurns;
extern volatile float gDebugMechanicalAngleRad;
extern volatile float gDebugElectricalAngleRad;
extern volatile float gFaultPhaseU;
extern volatile float gFaultPhaseV;
extern volatile float gFaultPhaseW;
extern volatile uint32_t gDebugIsrDeltaCycles;
extern volatile float gDebugIsrPeriodUs;
extern volatile float gDebugIsrFrequencyHz;
extern volatile uint8_t gIsrMeasureOnlyMode;
extern volatile float gIsrMeasureEdgeFrequencyHz;
extern volatile uint8_t gControlTimingMode;
extern volatile float gEffectiveCurrentLoopFrequencyHz;
extern volatile float gEffectiveSpeedLoopFrequencyHz;
extern uint16_t FaultCode;
extern float gTracePosError;
extern float RecordTable1[TRACE_DATA_LENGTH * 4u];
extern TraceData Trace_Data;
extern IdSquareTuning_t IdSquareTuning;
extern MotorAutoTune_t gMotorAutoTune;
extern void ApplyControlTimingMode(uint8_t mode);
extern volatile uint8_t gEncoderAlignmentPolicy;
extern volatile uint8_t gEncoderAlignmentStatus;
extern volatile uint8_t gEncoderAlignmentNeedsFlashSave;
extern volatile int32_t gEncoderAlignmentLastCapturedOffset;
extern volatile uint8_t gEncoderAlignmentRequested;
extern volatile uint8_t gEncoderAlignmentContinueToRun;
extern volatile uint8_t gEncoderAlignmentResumeRunMode;
extern volatile uint8_t gServoArmOnlyRequested;
extern volatile uint8_t gFocElectricalAngleTestMode;
extern volatile uint8_t gFocCurrentUvSwapTest;
extern volatile uint8_t gFocCurrentPolarityInvertTest;
extern volatile uint8_t gFocRotatingThetaTestRunning;
extern volatile uint8_t gFocRotatingThetaVoltageTestRunning;
extern volatile uint8_t gFocCurrentFeedbackMapTestRunning;
extern void PrepareEncoderAlignment(uint8_t continue_to_run, uint8_t resume_run_mode);
extern uint8_t SaveParametersToFlash(void);
extern uint8_t StartFocRotatingThetaTest(void);
extern uint8_t StartFocRotatingThetaVoltageTest(void);
extern uint8_t StartFocCurrentFeedbackMapTest(void);
extern void StopFocDiagnosticModes(void);

static void USB_StartServoSequence(uint8_t allow_auto_encoder_alignment);
static void USB_HandlePositionCommand(const uint8_t *payload, uint8_t payload_length);
static void USB_HandleFocControlStop(void);
static uint8_t USB_HandleStartFocRotatingThetaTest(void);
static uint8_t USB_HandleStartFocRotatingThetaVoltageTest(void);
static uint8_t USB_HandleStartFocCurrentFeedbackMapTest(void);
static uint8_t USB_GetCurrentCalibrationStatus(void);
static void USB_ResetTraceCaptureState(void);

static uint8_t USB_RingPush(uint8_t byte)
{
	uint16_t next = (uint16_t)((s_rx_head + 1u) % RX_BYTE_BUFFER_SIZE);
	if (next == s_rx_tail)
	{
		return USB_COMM_FAIL;
	}

	s_rx_buffer[s_rx_head] = byte;
	s_rx_head = next;
	return USB_COMM_OK;
}

static uint8_t USB_HandleStartFocRotatingThetaTest(void)
{
	if (StartFocRotatingThetaTest() == 0u)
	{
		return USB_COMM_FAIL;
	}
	if (Trace_Data.Enable != 0u)
	{
		USB_ResetTraceCaptureState();
	}
	return USB_COMM_OK;
}

static uint8_t USB_HandleStartFocRotatingThetaVoltageTest(void)
{
	if (StartFocRotatingThetaVoltageTest() == 0u)
	{
		return USB_COMM_FAIL;
	}
	if (Trace_Data.Enable != 0u)
	{
		USB_ResetTraceCaptureState();
	}
	return USB_COMM_OK;
}

static uint8_t USB_HandleStartFocCurrentFeedbackMapTest(void)
{
	if (StartFocCurrentFeedbackMapTest() == 0u)
	{
		return USB_COMM_FAIL;
	}
	if (Trace_Data.Enable != 0u)
	{
		USB_ResetTraceCaptureState();
	}
	return USB_COMM_OK;
}

static uint8_t USB_RingPop(uint8_t *byte)
{
	if (s_rx_head == s_rx_tail)
	{
		return USB_COMM_FAIL;
	}

	*byte = s_rx_buffer[s_rx_tail];
	s_rx_tail = (uint16_t)((s_rx_tail + 1u) % RX_BYTE_BUFFER_SIZE);
	return USB_COMM_OK;
}

static void USB_ResetParser(void)
{
	s_parser.index = 0u;
	s_parser.expected_length = 0u;
	s_parser.receiving = 0u;
}

static uint8_t USB_WriteRaw(const uint8_t *buffer, uint16_t length)
{
	uint32_t timeout = 50000u;

	while ((timeout > 0u) && (CDC_Transmit_FS((uint8_t *)buffer, length) == USBD_BUSY))
	{
		timeout--;
	}

	return (timeout == 0u) ? USB_COMM_FAIL : USB_COMM_OK;
}

static unsigned char CalcCRC(unsigned char uCode, unsigned char uSize, const unsigned char *buffer)
{
	unsigned char uCRC = 0u;
	unsigned short uSum = 0u;
	unsigned char index;

	(void)uCode;
	uSum += uSize;

	for (index = 0u; index < uSize; index++)
	{
		uSum += buffer[index];
	}

	uCRC = (unsigned char)(uSum & 0xFFu);
	uCRC += (unsigned char)(uSum >> 8);
	return uCRC;
}

void USB_QueueFocDebugText(const char *format, ...)
{
	va_list args;
	int written = 0;

	if (format == 0)
	{
		return;
	}
	if (s_foc_debug_pending != 0u)
	{
		return;
	}

	va_start(args, format);
	written = vsnprintf(
		(char *)s_foc_debug_buffer,
		sizeof(s_foc_debug_buffer),
		format,
		args);
	va_end(args);

	if (written <= 0)
	{
		return;
	}
	if (written >= (int)sizeof(s_foc_debug_buffer))
	{
		written = (int)sizeof(s_foc_debug_buffer) - 1;
	}

	s_foc_debug_length = (uint8_t)written;
	s_foc_debug_pending = 1u;
	USB_Comm.ReadFocDebugLog = 1u;
}

static void USB_SendAckPacket(uint8_t ack_code, uint8_t command, const uint8_t *payload, uint8_t payload_length)
{
	uint8_t frame[TX_FRAME_BUFFER_SIZE];
	uint16_t index = 0u;
	uint8_t size = (uint8_t)(payload_length + 1u);

	frame[index++] = STX;
	frame[index++] = ack_code;
	frame[index++] = size;
	frame[index++] = command;

	if ((payload_length > 0u) && (payload != 0))
	{
		memcpy(&frame[index], payload, payload_length);
		index = (uint16_t)(index + payload_length);
	}

	frame[index++] = CalcCRC(ack_code, size, &frame[3]);
	frame[index++] = ETX;
	(void)USB_WriteRaw(frame, index);
}

void SendData(UpdateDataCmd_e uCommand, uint16_t uDataLength, uint8_t *uSetData)
{
	uint8_t frame[TX_FRAME_BUFFER_SIZE];
	uint16_t index = 0u;
	uint8_t size = (uint8_t)(uDataLength + 1u);

	frame[index++] = STX;
	frame[index++] = SYN;
	frame[index++] = size;
	frame[index++] = (uint8_t)uCommand;

	if ((uDataLength > 0u) && (uSetData != 0))
	{
		memcpy(&frame[index], uSetData, uDataLength);
		index = (uint16_t)(index + uDataLength);
	}

	frame[index++] = CalcCRC(SYN, size, &frame[3]);
	frame[index++] = ETX;
	(void)USB_WriteRaw(frame, index);
}

static uint8_t USB_GetCurrentCalibrationStatus(void)
{
	if ((FaultCode & ERROR_CALIB_TIMEOUT) != 0u)
	{
		return CURRENT_CALIB_STATUS_TIMEOUT;
	}
	if ((FaultCode & ERROR_CURRENT_OFFSET_INVALID) != 0u)
	{
		return CURRENT_CALIB_STATUS_OFFSET_INVALID;
	}
	if (Current_Sensor.CalibFinish > 0)
	{
		return CURRENT_CALIB_STATUS_DONE;
	}
	if ((StateMachine.bState == OFFSET_CALIB) || (Current_Sensor.CalibCounter > 0u))
	{
		return CURRENT_CALIB_STATUS_RUNNING;
	}
	return CURRENT_CALIB_STATUS_IDLE;
}

static void USB_SendMonitorPacket(USB_Comunication_t *USB_Comunicate)
{
	uint8_t offset = 0u;
	float vdc_scaled = Parameter.fVdc * 0.1f;
	float cmd_speed = gCommandedSpeedRpm;
	float speed_error = cmd_speed - Parameter.fActSpeed;
	float cmd_position = 0.0f;
	float position_error = 0.0f;
	int16_t motor_power = (int16_t)(Parameter.fVdc * gIqRefA);
	uint8_t enable_run = (uint8_t)(((StateMachine.bState == RUN) || (StateMachine.bState == ENCODER_ALIGN)) ? 1u : 0u);
	uint16_t fault = FaultCode;
	uint8_t run_mode = (uint8_t)gRunMode;
	float id_ref = gIdRefA;
	uint8_t autotune_state = (uint8_t)gMotorAutoTune.state;
	uint8_t autotune_error = (uint8_t)gMotorAutoTune.error;
	uint8_t autotune_progress = gMotorAutoTune.progress_percent;
	uint8_t autotune_data_ready = gMotorAutoTune.tuning_data_ready;
	uint8_t foc_direction_test_status = FOC_DIRECTION_TEST_IDLE;
	int32_t foc_direction_test_open_loop_delta_pos = 0;
	int32_t foc_direction_test_foc_delta_pos = 0;
	uint16_t adc_offset_ia = Parameter.u16Offset_Ia;
	uint16_t adc_offset_ib = Parameter.u16Offset_Ib;
	uint8_t calibration_status = USB_GetCurrentCalibrationStatus();

	if ((((uint8_t)DriverParameter[CONTROL_MODE]) == POSITION_CONTROL_MODE) &&
		(run_mode == RUN_MODE_FOC))
	{
		cmd_position = gTargetPositionCounts;
		position_error = gTracePosError;
	}

	memcpy(&USB_Comunicate->TransmitData[offset], &enable_run, sizeof(enable_run));
	offset = (uint8_t)(offset + sizeof(enable_run));
	memcpy(&USB_Comunicate->TransmitData[offset], &vdc_scaled, sizeof(vdc_scaled));
	offset = (uint8_t)(offset + sizeof(vdc_scaled));
	memcpy(&USB_Comunicate->TransmitData[offset], &Parameter.fTemparature, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], &cmd_speed, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], &Parameter.fActSpeed, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], &speed_error, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], &cmd_position, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], &Parameter.fPosition, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], &position_error, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], &gIqRefA, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], &motor_power, sizeof(motor_power));
	offset = (uint8_t)(offset + sizeof(motor_power));
	memcpy(&USB_Comunicate->TransmitData[offset], &fault, sizeof(fault));
	offset = (uint8_t)(offset + sizeof(fault));
	memcpy(&USB_Comunicate->TransmitData[offset], &run_mode, sizeof(run_mode));
	offset = (uint8_t)(offset + sizeof(run_mode));
	memcpy(&USB_Comunicate->TransmitData[offset], &id_ref, sizeof(id_ref));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], &Parameter.fIabc[0], sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], &Parameter.fIabc[1], sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], &Parameter.fIabc[2], sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], &Parameter.fIdq[0], sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], &Parameter.fIdq[1], sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], &Parameter.fVdq[0], sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], &Parameter.fVdq[1], sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], &Parameter.fVabc[0], sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], &Parameter.fVabc[1], sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], &Parameter.fVabc[2], sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], &Parameter.fTheta, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gVfFrequencyHz, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gVfVoltageV, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gDebugOpenLoopElectricalHzCmd, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gDebugOpenLoopSyncRpmCmd, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gDebugObservedElectricalHz, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gDebugSpeedRawRpm, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gDebugExpectedDeltaPosSync, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gDebugDeltaPosAvg, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gDebugEncoderTurns, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gDebugMechanicalAngleRad, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gDebugElectricalAngleRad, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gDebugSpeedRawRpmAvg, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gDebugObservedElectricalHzAvg, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], &Parameter.DeltaPos, sizeof(Parameter.DeltaPos));
	offset = (uint8_t)(offset + sizeof(Parameter.DeltaPos));
	memcpy(&USB_Comunicate->TransmitData[offset], &Parameter.EncSingleTurn, sizeof(Parameter.EncSingleTurn));
	offset = (uint8_t)(offset + sizeof(Parameter.EncSingleTurn));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gDebugIsrDeltaCycles, sizeof(gDebugIsrDeltaCycles));
	offset = (uint8_t)(offset + sizeof(gDebugIsrDeltaCycles));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gDebugIsrPeriodUs, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gDebugIsrFrequencyHz, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gIsrMeasureOnlyMode, sizeof(gIsrMeasureOnlyMode));
	offset = (uint8_t)(offset + sizeof(gIsrMeasureOnlyMode));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gIsrMeasureEdgeFrequencyHz, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gControlTimingMode, sizeof(gControlTimingMode));
	offset = (uint8_t)(offset + sizeof(gControlTimingMode));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gEffectiveCurrentLoopFrequencyHz, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gEffectiveSpeedLoopFrequencyHz, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], &Parameter.Offset_Enc, sizeof(Parameter.Offset_Enc));
	offset = (uint8_t)(offset + sizeof(Parameter.Offset_Enc));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gEncoderAlignmentLastCapturedOffset, sizeof(gEncoderAlignmentLastCapturedOffset));
	offset = (uint8_t)(offset + sizeof(gEncoderAlignmentLastCapturedOffset));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gEncoderAlignmentPolicy, sizeof(gEncoderAlignmentPolicy));
	offset = (uint8_t)(offset + sizeof(gEncoderAlignmentPolicy));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gEncoderAlignmentStatus, sizeof(gEncoderAlignmentStatus));
	offset = (uint8_t)(offset + sizeof(gEncoderAlignmentStatus));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gEncoderAlignmentNeedsFlashSave, sizeof(gEncoderAlignmentNeedsFlashSave));
	offset = (uint8_t)(offset + sizeof(gEncoderAlignmentNeedsFlashSave));
	memcpy(&USB_Comunicate->TransmitData[offset], &autotune_state, sizeof(uint8_t));
	offset = (uint8_t)(offset + sizeof(uint8_t));
	memcpy(&USB_Comunicate->TransmitData[offset], &autotune_error, sizeof(uint8_t));
	offset = (uint8_t)(offset + sizeof(uint8_t));
	memcpy(&USB_Comunicate->TransmitData[offset], &autotune_progress, sizeof(uint8_t));
	offset = (uint8_t)(offset + sizeof(uint8_t));
	memcpy(&USB_Comunicate->TransmitData[offset], &autotune_data_ready, sizeof(uint8_t));
	offset = (uint8_t)(offset + sizeof(uint8_t));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gMotorAutoTune.measured_Rs, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gMotorAutoTune.measured_Ls, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gMotorAutoTune.measured_Ke, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gMotorAutoTune.measured_Flux, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gMotorAutoTune.measured_PolePairs, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gMotorAutoTune.tuned_current_kp, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gMotorAutoTune.tuned_current_ki, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gMotorAutoTune.tuned_speed_kp, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gMotorAutoTune.tuned_speed_ki, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gMotorAutoTune.tuned_position_kp, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], &foc_direction_test_status, sizeof(uint8_t));
	offset = (uint8_t)(offset + sizeof(uint8_t));
	memcpy(&USB_Comunicate->TransmitData[offset], &foc_direction_test_open_loop_delta_pos, sizeof(int32_t));
	offset = (uint8_t)(offset + sizeof(int32_t));
	memcpy(&USB_Comunicate->TransmitData[offset], &foc_direction_test_foc_delta_pos, sizeof(int32_t));
	offset = (uint8_t)(offset + sizeof(int32_t));
	memcpy(&USB_Comunicate->TransmitData[offset], &adc_offset_ia, sizeof(uint16_t));
	offset = (uint8_t)(offset + sizeof(uint16_t));
	memcpy(&USB_Comunicate->TransmitData[offset], &adc_offset_ib, sizeof(uint16_t));
	offset = (uint8_t)(offset + sizeof(uint16_t));
	memcpy(&USB_Comunicate->TransmitData[offset], &calibration_status, sizeof(uint8_t));
	offset = (uint8_t)(offset + sizeof(uint8_t));

	SendData(CMD_MONITOR_DATA, offset, USB_Comunicate->TransmitData);
	USB_Comunicate->ReadMotionMonitorData = 0u;
}

static void USB_SendFocDebugPacket(USB_Comunication_t *USB_Comunicate)
{
	if ((s_foc_debug_pending == 0u) || (s_foc_debug_length == 0u))
	{
		USB_Comunicate->ReadFocDebugLog = 0u;
		return;
	}

	SendData(CMD_FOC_DEBUG_TEXT, s_foc_debug_length, s_foc_debug_buffer);
	s_foc_debug_pending = 0u;
	s_foc_debug_length = 0u;
	USB_Comunicate->ReadFocDebugLog = 0u;
}

static void USB_SendErrorPacket(USB_Comunication_t *USB_Comunicate)
{
	uint8_t offset = 0u;
	int16_t motor_power = (int16_t)(Parameter.fVdc * gIqRefA);
	float cmd_speed = gCommandedSpeedRpm;
	float cmd_position = 0.0f;
	float position_error = 0.0f;

	if ((((uint8_t)DriverParameter[CONTROL_MODE]) == POSITION_CONTROL_MODE) &&
		(gRunMode == RUN_MODE_FOC))
	{
		cmd_position = gTargetPositionCounts;
		position_error = gTracePosError;
	}

	memcpy(&USB_Comunicate->TransmitData[offset], &FaultCode, sizeof(FaultCode));
	offset = (uint8_t)(offset + sizeof(FaultCode));
	memcpy(&USB_Comunicate->TransmitData[offset], &Parameter.fVdc, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], &Parameter.fTemparature, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], &cmd_speed, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], &Parameter.fActSpeed, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	{
		float speed_error = cmd_speed - Parameter.fActSpeed;
		memcpy(&USB_Comunicate->TransmitData[offset], &speed_error, sizeof(float));
		offset = (uint8_t)(offset + sizeof(float));
	}
	memcpy(&USB_Comunicate->TransmitData[offset], &cmd_position, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], &Parameter.fPosition, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], &position_error, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], &gIqRefA, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], &motor_power, sizeof(motor_power));
	offset = (uint8_t)(offset + sizeof(motor_power));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gFaultPhaseU, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gFaultPhaseV, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gFaultPhaseW, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));

SendData(MTR_CODE_ERROR, offset, USB_Comunicate->TransmitData);
	USB_Comunicate->SendError = false;
}

static void USB_SendTraceChunk(USB_Comunication_t *USB_Comunicate)
{
	if (Trace_Data.Finish == 0u)
	{
		return;
	}

	if (Trace_Data.u16Cnt2 < (TRACE_DATA_LENGTH / 10u))
	{
		uint16_t sample_index;
		uint8_t offset = 0u;

		memcpy(&USB_Comunicate->TransmitData[offset], &Trace_Data.u16Cnt2, sizeof(uint16_t));
		offset = (uint8_t)(offset + sizeof(uint16_t));

		for (sample_index = (uint16_t)(Trace_Data.u16Cnt2 * 10u);
			sample_index < (uint16_t)((Trace_Data.u16Cnt2 + 1u) * 10u);
			sample_index++)
		{
			uint16_t record_index = (uint16_t)(sample_index * 4u);
			memcpy(&USB_Comunicate->TransmitData[offset], &RecordTable1[record_index + 0u], sizeof(float));
			offset = (uint8_t)(offset + sizeof(float));
			memcpy(&USB_Comunicate->TransmitData[offset], &RecordTable1[record_index + 1u], sizeof(float));
			offset = (uint8_t)(offset + sizeof(float));
			memcpy(&USB_Comunicate->TransmitData[offset], &RecordTable1[record_index + 2u], sizeof(float));
			offset = (uint8_t)(offset + sizeof(float));
			memcpy(&USB_Comunicate->TransmitData[offset], &RecordTable1[record_index + 3u], sizeof(float));
			offset = (uint8_t)(offset + sizeof(float));
		}

		SendData(CMD_TRACE_DATA, offset, USB_Comunicate->TransmitData);
		Trace_Data.u16Cnt2++;
		return;
	}

	Trace_Data.u16Cnt2 = 0u;
	Trace_Data.Finish = 0u;
	if (Trace_Data.Mode == 0u)
	{
		Trace_Data.Enable = 0u;
		USB_Comunicate->ReadTraceData = 0u;
		if (USB_Comunicate->PriorityFlag == 1u)
		{
			USB_Comunicate->PriorityFlag = 0u;
		}
	}
	else
	{
		Trace_Data.Counter = 0u;
		Trace_Data.u8CntSample = Trace_Data.u8MaxCntSample;
	}
}

static void USB_SendAutoTuneChunk(USB_Comunication_t *USB_Comunicate)
{
	uint16_t start_index;
	uint16_t remaining;
	uint8_t valid_samples;
	uint8_t offset = 0u;
	uint16_t sample_index;
	static const uint8_t samples_per_chunk = 8u;

	if ((gMotorAutoTune.tuning_data_ready == 0u) ||
		(gMotorAutoTune.chart_transfer_active == 0u) ||
		(gMotorAutoTune.chart_length == 0u))
	{
		return;
	}

	start_index = gMotorAutoTune.chart_send_index;
	if (start_index >= gMotorAutoTune.chart_length)
	{
		gMotorAutoTune.chart_transfer_active = 0u;
		return;
	}

	remaining = (uint16_t)(gMotorAutoTune.chart_length - start_index);
	valid_samples = (remaining > samples_per_chunk) ? samples_per_chunk : (uint8_t)remaining;

	memcpy(&USB_Comunicate->TransmitData[offset], &gMotorAutoTune.chart_stage, sizeof(uint8_t));
	offset = (uint8_t)(offset + sizeof(uint8_t));
	memcpy(&USB_Comunicate->TransmitData[offset], &gMotorAutoTune.chart_send_index, sizeof(uint16_t));
	offset = (uint8_t)(offset + sizeof(uint16_t));
	memcpy(&USB_Comunicate->TransmitData[offset], &valid_samples, sizeof(uint8_t));
	offset = (uint8_t)(offset + sizeof(uint8_t));
	memcpy(&USB_Comunicate->TransmitData[offset], &gMotorAutoTune.chart_length, sizeof(uint16_t));
	offset = (uint8_t)(offset + sizeof(uint16_t));
	memcpy(&USB_Comunicate->TransmitData[offset], &gMotorAutoTune.chart_sample_period_s, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));

	for (sample_index = 0u; sample_index < samples_per_chunk; sample_index++)
	{
		float primary = 0.0f;
		float secondary = 0.0f;
		float tertiary = 0.0f;
		uint16_t data_index = (uint16_t)(start_index + sample_index);

		if (data_index < gMotorAutoTune.chart_length)
		{
			primary = gMotorAutoTune.chart_primary[data_index];
			secondary = gMotorAutoTune.chart_secondary[data_index];
			tertiary = gMotorAutoTune.chart_tertiary[data_index];
		}

		memcpy(&USB_Comunicate->TransmitData[offset], &primary, sizeof(float));
		offset = (uint8_t)(offset + sizeof(float));
		memcpy(&USB_Comunicate->TransmitData[offset], &secondary, sizeof(float));
		offset = (uint8_t)(offset + sizeof(float));
		memcpy(&USB_Comunicate->TransmitData[offset], &tertiary, sizeof(float));
		offset = (uint8_t)(offset + sizeof(float));
	}

	SendData(CMD_AUTOTUNING_DATA_THINH, offset, USB_Comunicate->TransmitData);
	gMotorAutoTune.chart_send_index = (uint16_t)(gMotorAutoTune.chart_send_index + valid_samples);
	if (gMotorAutoTune.chart_send_index >= gMotorAutoTune.chart_length)
	{
		gMotorAutoTune.chart_transfer_active = 0u;
	}
}

static void USB_SendParameterChunk(UpdateDataCmd_e code, const float *source, USB_Comunication_t *USB_Comunicate)
{
	uint8_t index;
	uint8_t count;
	uint8_t offset = 0u;
	uint8_t limit = USB_Comunicate->TotalLength;

	for (count = 0u; count < 20u; count++)
	{
		index = (uint8_t)(USB_Comunicate->Counter + count);
		if (index >= limit)
		{
			break;
		}

		memcpy(&USB_Comunicate->TransmitData[offset], &index, sizeof(index));
		offset = (uint8_t)(offset + sizeof(index));
		memcpy(&USB_Comunicate->TransmitData[offset], &source[index], sizeof(float));
		offset = (uint8_t)(offset + sizeof(float));
	}

	USB_Comunicate->TransmitData[offset++] = '\r';
	USB_Comunicate->TransmitData[offset++] = '\n';
	SendData(code, offset, USB_Comunicate->TransmitData);
	USB_Comunicate->Counter = (uint8_t)(USB_Comunicate->Counter + 20u);
}

static void USB_HandleReadDriver(USB_Comunication_t *USB_Comunicate, const uint8_t *payload, uint8_t payload_length)
{
	uint8_t total_length = (payload_length > 0u) ? payload[0] : DRIVER_PARAMETER_COUNT;
	if (total_length > DRIVER_PARAMETER_COUNT)
	{
		total_length = DRIVER_PARAMETER_COUNT;
	}

	USB_Comunicate->ReadDriverParameter = 1u;
	USB_Comunicate->ReadMotorParameter = 0u;
	USB_Comunicate->Counter = 0u;
	USB_Comunicate->TotalLength = total_length;
	USB_Comunicate->PriorityFlag = 2u;
}

static void USB_HandleReadMotor(USB_Comunication_t *USB_Comunicate, const uint8_t *payload, uint8_t payload_length)
{
	uint8_t total_length = (payload_length > 0u) ? payload[0] : 32u;
	if (total_length > 32u)
	{
		total_length = 32u;
	}

	USB_Comunicate->ReadDriverParameter = 0u;
	USB_Comunicate->ReadMotorParameter = 1u;
	USB_Comunicate->Counter = 0u;
	USB_Comunicate->TotalLength = total_length;
	USB_Comunicate->PriorityFlag = 2u;
}

static void USB_HandleWriteParameterPairs(float *target, const uint8_t *payload, uint8_t payload_length, uint8_t max_index)
{
	uint8_t offset = 0u;

	while ((offset + 5u) <= payload_length)
	{
		uint8_t index = payload[offset++];
		float value;
		memcpy(&value, &payload[offset], sizeof(float));
		offset = (uint8_t)(offset + sizeof(float));

		if (index < max_index)
		{
			target[index] = value;
		}
	}
}

static float *USB_ResolveTraceChannelPointer(uint8_t channel_code)
{
	switch (channel_code)
	{
		case 1u:
			return (float *)&gTargetSpeedRpm;
		case 2u:
			return &Parameter.fActSpeed;
		case 3u:
			return &gIqRefA;
		case 4u:
			return &Parameter.fIdq[1];
		case 5u:
			return &Parameter.fIabc[0];
		case 6u:
			return &Parameter.fIabc[1];
		case 7u:
			return &Parameter.fIabc[2];
		case 8u:
			return &Parameter.fVdc;
		case 9u:
			return &Parameter.fTemparature;
		case 10u:
			return &gTracePosError;
		case 11u:
			return &Parameter.fIdq[0];
		case 12u:
			return &gIdRefA;
		case 13u:
			return &Parameter.fVdq[0];
		case 14u:
			return &Parameter.fVdq[1];
		default:
			return 0;
	}
}

static void USB_ResetTraceCaptureState(void)
{
	Trace_Data.Counter = 0u;
	Trace_Data.u16Cnt2 = 0u;
	Trace_Data.Finish = 0u;
	Trace_Data.u8CntSample = Trace_Data.u8MaxCntSample;
}

static void USB_HandleTraceSetup(USB_Comunication_t *USB_Comunicate, const uint8_t *payload, uint8_t payload_length)
{
	if (payload_length < 7u)
	{
		return;
	}

	if (payload[0] == 0u)
	{
		Trace_Data.Enable = 0u;
		Trace_Data.Mode = 0u;
		Trace_Data.Finish = 0u;
		Trace_Data.Counter = 0u;
		Trace_Data.u16Cnt2 = 0u;
		Trace_Data.u8CntSample = 0u;
		Trace_Data.u8MaxCntSample = 0u;
		Trace_Data.Data1 = 0;
		Trace_Data.Data2 = 0;
		Trace_Data.Data3 = 0;
		Trace_Data.Data4 = 0;
		USB_Comunicate->ReadTraceData = 0u;
		if (USB_Comunicate->PriorityFlag == 1u)
		{
			USB_Comunicate->PriorityFlag = 0u;
		}
		return;
	}

	Trace_Data.Enable = 1u;
	Trace_Data.Mode = payload[1];
	Trace_Data.u8MaxCntSample = payload[6];
	Trace_Data.Data1 = USB_ResolveTraceChannelPointer(payload[2]);
	Trace_Data.Data2 = USB_ResolveTraceChannelPointer(payload[3]);
	Trace_Data.Data3 = USB_ResolveTraceChannelPointer(payload[4]);
	Trace_Data.Data4 = USB_ResolveTraceChannelPointer(payload[5]);
	USB_ResetTraceCaptureState();
	USB_Comunicate->ReadTraceData = 1u;
	USB_Comunicate->PriorityFlag = 1u;
}

static void USB_HandleIdSquareTuningConfig(const uint8_t *payload, uint8_t payload_length)
{
	float amplitude = 0.0f;
	float frequency = 10.0f;
	float current_kp;
	float current_ki;
	uint8_t electrical_angle_test_mode = 0u;
	uint8_t current_polarity_invert_test = 0u;
	uint8_t current_uv_swap_test = 0u;
	uint8_t tuning_mode = ID_SQUARE_TUNING_MODE_SQUARE_WAVE;

	if (payload_length < 16u)
	{
		return;
	}

	memcpy(&amplitude, &payload[0], sizeof(float));
	memcpy(&frequency, &payload[4], sizeof(float));
	memcpy(&current_kp, &payload[8], sizeof(float));
	memcpy(&current_ki, &payload[12], sizeof(float));
	if (payload_length >= 17u)
	{
		electrical_angle_test_mode = payload[16];
	}
	if (payload_length >= 18u)
	{
		current_polarity_invert_test = payload[17];
	}
	if (payload_length >= 19u)
	{
		current_uv_swap_test = payload[18];
	}
	if (payload_length >= 20u)
	{
		tuning_mode = payload[19];
	}

	IdSquareTuning.fAmplitudeCmd = amplitude;
	IdSquareTuning.fFrequencyCmd = frequency;
	IdSquareTuning.fCurrentKpCmd = current_kp;
	IdSquareTuning.fCurrentKiCmd = current_ki;
	IdSquareTuning.AlignmentDone = 0u;
	IdSquareTuning.AlignmentCounter = 0u;
	IdSquareTuning.ElectricalAngleTestMode = electrical_angle_test_mode;
	IdSquareTuning.CurrentPolarityInvertTest = (current_polarity_invert_test != 0u) ? 1u : 0u;
	IdSquareTuning.CurrentUvSwapTest = (current_uv_swap_test != 0u) ? 1u : 0u;
	IdSquareTuning.Mode = (tuning_mode == ID_SQUARE_TUNING_MODE_ALIGNMENT_HOLD) ?
		ID_SQUARE_TUNING_MODE_ALIGNMENT_HOLD : ID_SQUARE_TUNING_MODE_SQUARE_WAVE;
	IdSquareTuning.fPhase = 0.0f;
	IdSquareTuning.fAlignmentCurrentApplied = 0.0f;
	IdSquareTuning.OffsetCaptured = 0;
}

static void USB_HandleIdSquareTuningStart(void)
{
	uint8_t reuse_completed_alignment = 0u;

	if ((StateMachine.bState != IDLE) && (StateMachine.bState != STOP))
	{
		IdSquareTuning.Enable = 0u;
		IdSquareTuning.AlignmentDone = 0u;
		IdSquareTuning.AlignmentCounter = 0u;
		IdSquareTuning.fPhase = 0.0f;
		IdSquareTuning.fAlignmentCurrentApplied = 0.0f;
		IdSquareTuning.OffsetCaptured = 0;
		return;
	}

	gRunMode = (IdSquareTuning.Mode == ID_SQUARE_TUNING_MODE_ALIGNMENT_HOLD) ?
		RUN_MODE_ALIGNMENT_ONLY : RUN_MODE_FOC;
	gVfFrequencyHz = 0.0f;
	gVfVoltageV = 0.0f;
	gTargetSpeedRpm = 0.0f;
	gIdRefA = 0.0f;
	gIqRefA = 0.0f;
	if ((IdSquareTuning.Mode == ID_SQUARE_TUNING_MODE_SQUARE_WAVE) &&
		(gEncoderAlignmentStatus == ENCODER_ALIGNMENT_STATUS_DONE))
	{
		reuse_completed_alignment = 1u;
	}
	/* Square-wave Id commissioning is intended to run on a fixed electrical
	   frame. Keep that frame deterministic between runs instead of following
	   the live encoder angle after the user presses Start. */
	IdSquareTuning.Enable = 1u;
	IdSquareTuning.AlignmentDone = reuse_completed_alignment;
	IdSquareTuning.AlignmentCounter = 0u;
	IdSquareTuning.fPhase = 0.0f;
	IdSquareTuning.fVoltageLimitApplied = 0.0f;
	IdSquareTuning.fAlignmentCurrentApplied = 0.0f;
	IdSquareTuning.OffsetCaptured = reuse_completed_alignment ? Parameter.Offset_Enc : 0;
	gServoArmOnlyRequested = 0u;
	USB_StartServoSequence(0u);
	if (Trace_Data.Enable != 0u)
	{
		USB_ResetTraceCaptureState();
	}
}

static void USB_HandleIdSquareTuningStop(void)
{
	IdSquareTuning.Enable = 0u;
	IdSquareTuning.AlignmentDone = 0u;
	IdSquareTuning.AlignmentCounter = 0u;
	IdSquareTuning.fPhase = 0.0f;
	IdSquareTuning.fVoltageLimitApplied = 0.0f;
	IdSquareTuning.fAlignmentCurrentApplied = 0.0f;
	IdSquareTuning.OffsetCaptured = 0;
	gRunMode = RUN_MODE_FOC;
	gVfFrequencyHz = 0.0f;
	gVfVoltageV = 0.0f;
	gTargetSpeedRpm = 0.0f;
	gIdRefA = 0.0f;
	gIqRefA = 0.0f;
	gEncoderAlignmentRequested = 0u;
	gEncoderAlignmentContinueToRun = 0u;
	gEncoderAlignmentResumeRunMode = RUN_MODE_FOC;
	if ((gEncoderAlignmentStatus == ENCODER_ALIGNMENT_STATUS_REQUESTED) ||
		(gEncoderAlignmentStatus == ENCODER_ALIGNMENT_STATUS_RUNNING))
	{
		gEncoderAlignmentStatus = ENCODER_ALIGNMENT_STATUS_IDLE;
	}
	STM_NextState(&StateMachine, STOP);
}

static void USB_StartServoSequence(uint8_t allow_auto_encoder_alignment)
{
	Reset_CurrentSensor(&Current_Sensor);
	FaultCode = NO_ERROR;
	USB_Comm.SendError = false;
	if ((allow_auto_encoder_alignment != 0u) &&
		(IdSquareTuning.Enable == 0u) &&
		(gEncoderAlignmentRequested == 0u) &&
		(gEncoderAlignmentPolicy == ENCODER_ALIGNMENT_POLICY_POWER_ON))
	{
		PrepareEncoderAlignment(1u, gRunMode);
	}
	STM_NextState(&StateMachine, OFFSET_CALIB);
}

static uint8_t USB_HandleStartEncoderAlignment(void)
{
	if ((StateMachine.bState != IDLE) && (StateMachine.bState != STOP))
	{
		return USB_COMM_FAIL;
	}

	PrepareEncoderAlignment(0u, RUN_MODE_FOC);
	gServoArmOnlyRequested = 0u;
	USB_StartServoSequence(0u);
	if (Trace_Data.Enable != 0u)
	{
		USB_ResetTraceCaptureState();
	}
	return USB_COMM_OK;
}

static uint8_t USB_HandleWriteToFlash(void)
{
	return (SaveParametersToFlash() != 0u) ? USB_COMM_OK : USB_COMM_FAIL;
}

static void USB_HandleSpeedCommand(const uint8_t *payload, uint8_t payload_length)
{
	float speed_limit_rpm = DriverParameter[MAXIMUM_SPEED];
	float motor_max_speed_rpm = MotorParameter[MOTOR_MAXIMUM_SPEED];
	uint8_t foc_angle_test_mode = ID_SQUARE_ANGLE_TEST_NONE;
	uint8_t foc_current_uv_swap_test = 0u;

	StopFocDiagnosticModes();
	gRunMode = RUN_MODE_FOC;
	gVfFrequencyHz = 0.0f;
	gVfVoltageV = 0.0f;
	DriverParameter[CONTROL_MODE] = (float)SPEED_CONTROL_MODE;
	IdSquareTuning.Enable = 0u;
	IdSquareTuning.fPhase = 0.0f;
	IdSquareTuning.fVoltageLimitApplied = 0.0f;
	ResetControl_V_over_F();
	gServoArmOnlyRequested = 0u;

	if (payload_length >= sizeof(float))
	{
		memcpy((void *)&gTargetSpeedRpm, payload, sizeof(float));
	}
	if (payload_length >= 12u)
	{
		memcpy(&DriverParameter[SPEED_P_GAIN], &payload[4], sizeof(float));
		memcpy(&DriverParameter[SPEED_I_GAIN], &payload[8], sizeof(float));
	}
	if (payload_length >= 20u)
	{
		memcpy(&DriverParameter[ACCELERATION_TIME], &payload[12], sizeof(float));
		memcpy(&DriverParameter[DECELERATION_TIME], &payload[16], sizeof(float));
	}
	if (payload_length >= 24u)
	{
		memcpy(&speed_limit_rpm, &payload[20], sizeof(float));
	}
	if (payload_length >= 25u)
	{
		foc_angle_test_mode = payload[24];
	}
	else if (payload_length >= 21u)
	{
		foc_angle_test_mode = payload[20];
	}
	if (payload_length >= 26u)
	{
		foc_current_uv_swap_test = payload[25];
	}
	else if (payload_length >= 22u)
	{
		foc_current_uv_swap_test = payload[21];
	}

	if (speed_limit_rpm < 0.0f)
	{
		speed_limit_rpm = -speed_limit_rpm;
	}
	if (motor_max_speed_rpm > 0.0f)
	{
		if ((speed_limit_rpm <= 0.0f) || (speed_limit_rpm > motor_max_speed_rpm))
		{
			speed_limit_rpm = motor_max_speed_rpm;
		}
	}
	if (speed_limit_rpm > 0.0f)
	{
		DriverParameter[MAXIMUM_SPEED] = speed_limit_rpm;
	}
	UpdateDriverParameter(DriverParameter);
	gFocElectricalAngleTestMode = foc_angle_test_mode;
	gFocCurrentUvSwapTest = (foc_current_uv_swap_test != 0u) ? 1u : 0u;

	if (DriverParameter[MAXIMUM_SPEED] > 0.0f)
	{
		if (gTargetSpeedRpm > DriverParameter[MAXIMUM_SPEED])
		{
			gTargetSpeedRpm = DriverParameter[MAXIMUM_SPEED];
		}
		if (gTargetSpeedRpm < -DriverParameter[MAXIMUM_SPEED])
		{
			gTargetSpeedRpm = -DriverParameter[MAXIMUM_SPEED];
		}
	}

	gTargetPositionCounts = Parameter.fPosition;
	gCommandedSpeedRpm = Parameter.fActSpeed;
	gTracePosError = 0.0f;
	USB_QueueFocDebugText(
		"[FOC] speed start tgt=%.1f max=%.1f kp=%.3f ki=%.3f",
		gTargetSpeedRpm,
		DriverParameter[MAXIMUM_SPEED],
		DriverParameter[SPEED_P_GAIN],
		DriverParameter[SPEED_I_GAIN]);
}

static void USB_HandlePositionCommand(const uint8_t *payload, uint8_t payload_length)
{
	float requested_position = Parameter.fPosition;
	float speed_limit_rpm = DriverParameter[MAXIMUM_SPEED];
	float motor_max_speed_rpm = MotorParameter[MOTOR_MAXIMUM_SPEED];
	uint8_t foc_angle_test_mode = ID_SQUARE_ANGLE_TEST_NONE;
	uint8_t foc_current_uv_swap_test = 0u;
	uint8_t position_tracking_mode = (uint8_t)DriverParameter[POSITION_TRACKING_MODE];

	StopFocDiagnosticModes();
	gRunMode = RUN_MODE_FOC;
	gVfFrequencyHz = 0.0f;
	gVfVoltageV = 0.0f;
	DriverParameter[CONTROL_MODE] = (float)POSITION_CONTROL_MODE;
	IdSquareTuning.Enable = 0u;
	IdSquareTuning.fPhase = 0.0f;
	IdSquareTuning.fVoltageLimitApplied = 0.0f;
	ResetControl_V_over_F();
	gServoArmOnlyRequested = 0u;

	if (payload_length >= 4u)
	{
		memcpy(&requested_position, &payload[0], sizeof(float));
	}
	if (payload_length >= 8u)
	{
		memcpy(&speed_limit_rpm, &payload[4], sizeof(float));
	}
	if (payload_length >= 12u)
	{
		memcpy(&DriverParameter[POSITION_P_GAIN], &payload[8], sizeof(float));
	}
	if (payload_length >= 16u)
	{
		memcpy(&DriverParameter[POSITION_I_GAIN], &payload[12], sizeof(float));
	}
	if (payload_length >= 20u)
	{
		memcpy(&DriverParameter[POSITION_FF_GAIN], &payload[16], sizeof(float));
	}
	if (payload_length >= 24u)
	{
		memcpy(&DriverParameter[POSITION_FF_FILTER], &payload[20], sizeof(float));
	}
	if (payload_length >= 32u)
	{
		memcpy(&DriverParameter[SPEED_P_GAIN], &payload[24], sizeof(float));
		memcpy(&DriverParameter[SPEED_I_GAIN], &payload[28], sizeof(float));
	}
	else if (payload_length >= 20u)
	{
		memcpy(&DriverParameter[SPEED_P_GAIN], &payload[12], sizeof(float));
		memcpy(&DriverParameter[SPEED_I_GAIN], &payload[16], sizeof(float));
	}
	if (payload_length >= 40u)
	{
		memcpy(&DriverParameter[ACCELERATION_TIME], &payload[32], sizeof(float));
		memcpy(&DriverParameter[DECELERATION_TIME], &payload[36], sizeof(float));
	}
	else if (payload_length >= 28u)
	{
		memcpy(&DriverParameter[ACCELERATION_TIME], &payload[20], sizeof(float));
		memcpy(&DriverParameter[DECELERATION_TIME], &payload[24], sizeof(float));
	}
	if (payload_length >= 41u)
	{
		foc_angle_test_mode = payload[40];
	}
	else if (payload_length >= 29u)
	{
		foc_angle_test_mode = payload[28];
	}
	if (payload_length >= 42u)
	{
		foc_current_uv_swap_test = payload[41];
	}
	else if (payload_length >= 30u)
	{
		foc_current_uv_swap_test = payload[29];
	}
	if (payload_length >= 43u)
	{
		position_tracking_mode = payload[42];
	}
	if (position_tracking_mode > POSITION_TRACKING_MODE_MULTI_TURN)
	{
		position_tracking_mode = POSITION_TRACKING_MODE_SINGLE_TURN;
	}

	if (speed_limit_rpm < 0.0f)
	{
		speed_limit_rpm = -speed_limit_rpm;
	}
	if (motor_max_speed_rpm > 0.0f)
	{
		if ((speed_limit_rpm <= 0.0f) || (speed_limit_rpm > motor_max_speed_rpm))
		{
			speed_limit_rpm = motor_max_speed_rpm;
		}
	}
	if (speed_limit_rpm > 0.0f)
	{
		DriverParameter[MAXIMUM_SPEED] = speed_limit_rpm;
	}
	DriverParameter[POSITION_TRACKING_MODE] = (float)position_tracking_mode;
	UpdateDriverParameter(DriverParameter);
	gFocElectricalAngleTestMode = foc_angle_test_mode;
	gFocCurrentUvSwapTest = (foc_current_uv_swap_test != 0u) ? 1u : 0u;

	gTargetPositionCounts = requested_position;
	gTargetSpeedRpm = DriverParameter[MAXIMUM_SPEED];
	gCommandedSpeedRpm = 0.0f;
	gTracePosError = gTargetPositionCounts - Parameter.fPosition;
	USB_QueueFocDebugText(
		"[FOC] pos start tgt=%.1f lim=%.1f act=%.1f err=%.1f kp=%.3f ki=%.3f vff=%.3f vf=%.1f mode=%s",
		gTargetPositionCounts,
		DriverParameter[MAXIMUM_SPEED],
		Parameter.fPosition,
		gTracePosError,
		DriverParameter[POSITION_P_GAIN],
		DriverParameter[POSITION_I_GAIN],
		DriverParameter[POSITION_FF_GAIN],
		DriverParameter[POSITION_FF_FILTER],
		(position_tracking_mode == POSITION_TRACKING_MODE_MULTI_TURN) ? "multi" : "single");
}

static void USB_HandleFocControlStop(void)
{
	gRunMode = RUN_MODE_FOC;
	gVfFrequencyHz = 0.0f;
	gVfVoltageV = 0.0f;
	gFocElectricalAngleTestMode = ID_SQUARE_ANGLE_TEST_NONE;
	gFocCurrentUvSwapTest = 0u;
	gFocCurrentPolarityInvertTest = 0u;
	gFocRotatingThetaTestRunning = 0u;
	gFocRotatingThetaVoltageTestRunning = 0u;
	gFocCurrentFeedbackMapTestRunning = 0u;
	ResetControl_V_over_F();
	IdSquareTuning.Enable = 0u;
	IdSquareTuning.fPhase = 0.0f;
	IdSquareTuning.fVoltageLimitApplied = 0.0f;
	gTargetSpeedRpm = 0.0f;
	gTargetPositionCounts = Parameter.fPosition;
	gCommandedSpeedRpm = 0.0f;
	gTracePosError = 0.0f;
	gIdRefA = 0.0f;
	gIqRefA = 0.0f;
	gEncoderAlignmentRequested = 0u;
	gEncoderAlignmentContinueToRun = 0u;
	gEncoderAlignmentResumeRunMode = RUN_MODE_FOC;
	gServoArmOnlyRequested = 0u;
	if ((gEncoderAlignmentStatus == ENCODER_ALIGNMENT_STATUS_REQUESTED) ||
		(gEncoderAlignmentStatus == ENCODER_ALIGNMENT_STATUS_RUNNING))
	{
		gEncoderAlignmentStatus = ENCODER_ALIGNMENT_STATUS_IDLE;
	}
	STM_NextState(&StateMachine, STOP);
}

static void USB_HandleSpeedTuning(const uint8_t *payload, uint8_t payload_length)
{
	if (payload_length >= 16u)
	{
		memcpy((void *)&gTargetSpeedRpm, &payload[0], sizeof(float));
		memcpy(&DriverParameter[SPEED_P_GAIN], &payload[8], sizeof(float));
		memcpy(&DriverParameter[SPEED_I_GAIN], &payload[12], sizeof(float));
		UpdateDriverParameter(DriverParameter);
		return;
	}

	if (payload_length >= 8u)
	{
		memcpy(&DriverParameter[SPEED_P_GAIN], &payload[0], sizeof(float));
		memcpy(&DriverParameter[SPEED_I_GAIN], &payload[4], sizeof(float));
		UpdateDriverParameter(DriverParameter);
		return;
	}

	if (payload_length >= 4u)
	{
		memcpy((void *)&gTargetSpeedRpm, &payload[0], sizeof(float));
	}
}

static void USB_HandleOpenLoopVfCommand(const uint8_t *payload, uint8_t payload_length)
{
	float requested_frequency = 0.0f;
	float requested_voltage = 0.0f;

	if (payload_length >= 8u)
	{
		memcpy(&requested_frequency, &payload[0], sizeof(float));
		memcpy(&requested_voltage, &payload[4], sizeof(float));
	}

	if (requested_frequency > 50.0f)
	{
		requested_frequency = 50.0f;
	}
	if (requested_frequency < -50.0f)
	{
		requested_frequency = -50.0f;
	}
	if (requested_voltage < 0.0f)
	{
		requested_voltage = 0.0f;
	}
	if (requested_voltage > 40.0f)
	{
		requested_voltage = 40.0f;
	}

	gVfFrequencyHz = requested_frequency;
	gVfVoltageV = requested_voltage;
	gTargetSpeedRpm = 0.0f;
	gTargetPositionCounts = Parameter.fPosition;
	gCommandedSpeedRpm = 0.0f;
	gTracePosError = 0.0f;
	gIdRefA = 0.0f;
	gIqRefA = 0.0f;
	IdSquareTuning.Enable = 0u;
	IdSquareTuning.fPhase = 0.0f;
	IdSquareTuning.fVoltageLimitApplied = 0.0f;
	gRunMode = RUN_MODE_OPEN_LOOP_VF;
	gServoArmOnlyRequested = 0u;
	ResetControl_V_over_F();
}

static void USB_HandleFaultAck(void)
{
	FaultCode = NO_ERROR;
	USB_Comm.SendError = false;
	gRunMode = RUN_MODE_FOC;
	gVfFrequencyHz = 0.0f;
	gVfVoltageV = 0.0f;
	gFocCurrentUvSwapTest = 0u;
	gFocCurrentPolarityInvertTest = 0u;
	gFocRotatingThetaTestRunning = 0u;
	gFocRotatingThetaVoltageTestRunning = 0u;
	gFocCurrentFeedbackMapTestRunning = 0u;
	gTargetSpeedRpm = 0.0f;
	gTargetPositionCounts = Parameter.fPosition;
	gCommandedSpeedRpm = 0.0f;
	gTracePosError = 0.0f;
	gIdRefA = 0.0f;
	gIqRefA = 0.0f;
	IdSquareTuning.Enable = 0u;
	IdSquareTuning.fPhase = 0.0f;
	IdSquareTuning.fVoltageLimitApplied = 0.0f;
	ResetControl_V_over_F();
	STM_FaultProcessing(&StateMachine, 0u, 0u);
	gEncoderAlignmentRequested = 0u;
	gEncoderAlignmentContinueToRun = 0u;
	gEncoderAlignmentResumeRunMode = RUN_MODE_FOC;
	gServoArmOnlyRequested = 0u;
	if ((gEncoderAlignmentStatus == ENCODER_ALIGNMENT_STATUS_REQUESTED) ||
		(gEncoderAlignmentStatus == ENCODER_ALIGNMENT_STATUS_RUNNING))
	{
		gEncoderAlignmentStatus = ENCODER_ALIGNMENT_STATUS_IDLE;
	}
	STM_NextState(&StateMachine, IDLE);
}

static void USB_HandleServoOn(void)
{
	gRunMode = RUN_MODE_FOC;
	gVfFrequencyHz = 0.0f;
	gVfVoltageV = 0.0f;
	gFocElectricalAngleTestMode = ID_SQUARE_ANGLE_TEST_NONE;
	gFocCurrentUvSwapTest = 0u;
	gFocCurrentPolarityInvertTest = 0u;
	gFocRotatingThetaTestRunning = 0u;
	gFocRotatingThetaVoltageTestRunning = 0u;
	gFocCurrentFeedbackMapTestRunning = 0u;
	gTargetSpeedRpm = 0.0f;
	gTargetPositionCounts = Parameter.fPosition;
	gCommandedSpeedRpm = 0.0f;
	gTracePosError = 0.0f;
	gIdRefA = 0.0f;
	gIqRefA = 0.0f;
	IdSquareTuning.Enable = 0u;
	IdSquareTuning.fPhase = 0.0f;
	IdSquareTuning.fVoltageLimitApplied = 0.0f;
	gEncoderAlignmentRequested = 0u;
	gEncoderAlignmentContinueToRun = 0u;
	gEncoderAlignmentResumeRunMode = RUN_MODE_FOC;
	gServoArmOnlyRequested = 0u;
	PrepareEncoderAlignment(0u, RUN_MODE_FOC);
	ResetControl_V_over_F();
	USB_StartServoSequence(0u);
}

static void USB_HandleServoOff(void)
{
	USB_HandleFocControlStop();
}

static void USB_HandleOpenLoopStop(void)
{
	USB_HandleFocControlStop();
}

static void USB_HandleControlTimingMode(const uint8_t *payload, uint8_t payload_length)
{
	(void)payload;
	(void)payload_length;
	ApplyControlTimingMode(CONTROL_TIMING_MODE_16KHZ);
}

static uint8_t USB_HandleAutoTuneStart(USB_Comunication_t *USB_Comunicate, const uint8_t *payload, uint8_t payload_length)
{
	MotorAutoTuneConfig_t config;

	if ((StateMachine.bState != IDLE) && (StateMachine.bState != STOP))
	{
		return USB_COMM_FAIL;
	}

	MotorAutoTune_SetDefaultConfig(&config);
	if (payload_length >= 32u)
	{
		memcpy(&config.rs_current_low_a, &payload[0], sizeof(float));
		memcpy(&config.rs_current_high_a, &payload[4], sizeof(float));
		memcpy(&config.ls_step_voltage_v, &payload[8], sizeof(float));
		memcpy(&config.flux_frequency_hz, &payload[12], sizeof(float));
		memcpy(&config.flux_voltage_v, &payload[16], sizeof(float));
		memcpy(&config.current_bandwidth_hz, &payload[20], sizeof(float));
		memcpy(&config.speed_bandwidth_hz, &payload[24], sizeof(float));
		memcpy(&config.position_bandwidth_hz, &payload[28], sizeof(float));
	}

	if (MotorAutoTune_Start(
		&gMotorAutoTune,
		&config,
		gEffectiveCurrentLoopFrequencyHz,
		Current_Sensor.OverCurrentThreshold) == 0u)
	{
		return USB_COMM_FAIL;
	}

	gRunMode = RUN_MODE_AUTOTUNE;
	gTargetSpeedRpm = 0.0f;
	gTargetPositionCounts = Parameter.fPosition;
	gCommandedSpeedRpm = 0.0f;
	gTracePosError = 0.0f;
	gIdRefA = 0.0f;
	gIqRefA = 0.0f;
	gVfFrequencyHz = 0.0f;
	gVfVoltageV = 0.0f;
	IdSquareTuning.Enable = 0u;
	IdSquareTuning.AlignmentDone = 0u;
	IdSquareTuning.AlignmentCounter = 0u;
	IdSquareTuning.fPhase = 0.0f;
	IdSquareTuning.fVoltageLimitApplied = 0.0f;
	gServoArmOnlyRequested = 0u;
	ResetControl_V_over_F();
	USB_Comunicate->ReadAutoTuningData_T = 0u;
	if (Trace_Data.Enable != 0u)
	{
		USB_ResetTraceCaptureState();
	}
	USB_StartServoSequence(1u);
	return USB_COMM_OK;
}

static void USB_HandleAutoTuneStop(USB_Comunication_t *USB_Comunicate)
{
	MotorAutoTune_Stop(&gMotorAutoTune);
	USB_Comunicate->ReadAutoTuningData_T = 0u;
	gRunMode = RUN_MODE_FOC;
	gTargetSpeedRpm = 0.0f;
	gTargetPositionCounts = Parameter.fPosition;
	gCommandedSpeedRpm = 0.0f;
	gTracePosError = 0.0f;
	gIdRefA = 0.0f;
	gIqRefA = 0.0f;
	gVfFrequencyHz = 0.0f;
	gVfVoltageV = 0.0f;
	ResetControl_V_over_F();
	STM_NextState(&StateMachine, STOP);
}

static uint8_t USB_HandleAutoTuneApplyGains(void)
{
	if (MotorAutoTune_ApplyEstimatedParameters(
		&gMotorAutoTune,
		DriverParameter,
		DRIVER_PARAMETER_COUNT,
		MotorParameter,
		32u) == 0u)
	{
		return USB_COMM_FAIL;
	}

	UpdateDriverParameter(DriverParameter);
	UpdateMotorParameter(MotorParameter);
	return USB_COMM_OK;
}

static void USB_HandleAutoTuneContinue(void)
{
	MotorAutoTune_ClearDataReady(&gMotorAutoTune);
}

static void USB_DispatchCommand(USB_Comunication_t *USB_Comunicate, uint8_t command, const uint8_t *payload, uint8_t payload_length)
{
	switch (command)
	{
		case CMD_SERVO_ON:
			USB_SendAckPacket(ACK, command, payload, payload_length);
			USB_HandleServoOn();
			break;

		case CMD_SERVO_OFF:
			USB_SendAckPacket(ACK, command, payload, payload_length);
			USB_HandleServoOff();
			break;

		case CMD_START_SPEEDCONTROL:
			USB_SendAckPacket(ACK, command, payload, payload_length);
			USB_HandleSpeedCommand(payload, payload_length);
			if ((StateMachine.bState == IDLE) || (StateMachine.bState == STOP))
			{
				USB_StartServoSequence(0u);
			}
			break;

		case CMD_STOP_SPEEDCONTROL:
			USB_SendAckPacket(ACK, command, payload, payload_length);
			USB_HandleFocControlStop();
			break;

		case CMD_START_POSITIONCONTROL:
			USB_SendAckPacket(ACK, command, payload, payload_length);
			USB_HandlePositionCommand(payload, payload_length);
			if ((StateMachine.bState == IDLE) || (StateMachine.bState == STOP))
			{
				USB_StartServoSequence(0u);
			}
			break;

		case CMD_STOP_POSITIONCONTROL:
			USB_SendAckPacket(ACK, command, payload, payload_length);
			USB_HandleFocControlStop();
			break;

		case CMD_APPLY_STUNING:
			USB_SendAckPacket(ACK, command, payload, payload_length);
			USB_HandleSpeedTuning(payload, payload_length);
			break;

		case CMD_START_OPEN_LOOP_VF:
			USB_SendAckPacket(ACK, command, payload, payload_length);
			USB_HandleOpenLoopVfCommand(payload, payload_length);
			if ((StateMachine.bState == IDLE) || (StateMachine.bState == STOP))
			{
				USB_StartServoSequence(0u);
			}
			break;

		case CMD_STOP_OPEN_LOOP_VF:
			USB_SendAckPacket(ACK, command, payload, payload_length);
			USB_HandleOpenLoopStop();
			break;

		case CMD_APPLY_TRACE:
			USB_SendAckPacket(ACK, command, payload, payload_length);
			USB_HandleTraceSetup(USB_Comunicate, payload, payload_length);
			break;

		case CMD_APPLY_ID_SQUARE_TUNING:
			USB_SendAckPacket(ACK, command, payload, payload_length);
			USB_HandleIdSquareTuningConfig(payload, payload_length);
			break;

		case CMD_START_ID_SQUARE_TUNING:
			USB_SendAckPacket(ACK, command, payload, payload_length);
			USB_HandleIdSquareTuningStart();
			break;

		case CMD_STOP_ID_SQUARE_TUNING:
			USB_SendAckPacket(ACK, command, payload, payload_length);
			USB_HandleIdSquareTuningStop();
			break;

		case CMD_SET_CONTROL_TIMING_MODE:
			USB_SendAckPacket(ACK, command, payload, payload_length);
			USB_HandleControlTimingMode(payload, payload_length);
			break;

		case CMD_START_AUTOTUNING_T:
			if (USB_HandleAutoTuneStart(USB_Comunicate, payload, payload_length) == USB_COMM_OK)
			{
				USB_SendAckPacket(ACK, command, payload, payload_length);
			}
			else
			{
				USB_SendAckPacket(ACK_ERROR, command, payload, payload_length);
			}
			break;

		case CMD_STOP_AUTOTUNING_T:
			USB_SendAckPacket(ACK, command, payload, payload_length);
			USB_HandleAutoTuneStop(USB_Comunicate);
			break;

		case CMD_UPDATE_TUNING_GAIN:
			if (USB_HandleAutoTuneApplyGains() == USB_COMM_OK)
			{
				USB_SendAckPacket(ACK, command, payload, payload_length);
			}
			else
			{
				USB_SendAckPacket(ACK_ERROR, command, payload, payload_length);
			}
			break;

		case CMD_CONTINUE_AUTO_TUNING_STATE:
			USB_SendAckPacket(ACK, command, payload, payload_length);
			USB_HandleAutoTuneContinue();
			break;

		case CMD_START_ENCODER_ALIGNMENT:
			if (USB_HandleStartEncoderAlignment() == USB_COMM_OK)
			{
				USB_SendAckPacket(ACK, command, payload, payload_length);
			}
			else
			{
				USB_SendAckPacket(ACK_ERROR, command, payload, payload_length);
			}
			break;

		case CMD_START_FOC_DIRECTION_TEST:
			USB_SendAckPacket(ACK_ERROR, command, payload, payload_length);
			break;

		case CMD_START_FOC_ANGLE_FIT:
			USB_SendAckPacket(ACK_ERROR, command, payload, payload_length);
			break;

		case CMD_START_FOC_ROTATING_THETA_TEST:
			if (USB_HandleStartFocRotatingThetaTest() == USB_COMM_OK)
			{
				USB_SendAckPacket(ACK, command, payload, payload_length);
			}
			else
			{
				USB_SendAckPacket(ACK_ERROR, command, payload, payload_length);
			}
			break;

		case CMD_START_FOC_ROTATING_THETA_VOLTAGE_TEST:
			if (USB_HandleStartFocRotatingThetaVoltageTest() == USB_COMM_OK)
			{
				USB_SendAckPacket(ACK, command, payload, payload_length);
			}
			else
			{
				USB_SendAckPacket(ACK_ERROR, command, payload, payload_length);
			}
			break;

		case CMD_START_FOC_CURRENT_FEEDBACK_MAP_TEST:
			if (USB_HandleStartFocCurrentFeedbackMapTest() == USB_COMM_OK)
			{
				USB_SendAckPacket(ACK, command, payload, payload_length);
			}
			else
			{
				USB_SendAckPacket(ACK_ERROR, command, payload, payload_length);
			}
			break;

		case CMD_WRITE_TO_FLASH:
			if (USB_HandleWriteToFlash() == USB_COMM_OK)
			{
				USB_SendAckPacket(ACK, command, payload, payload_length);
			}
			else
			{
				USB_SendAckPacket(ACK_ERROR, command, payload, payload_length);
			}
			break;

		case CMD_UPDATE_MONITOR:
			USB_SendAckPacket(ACK, command, payload, payload_length);
			USB_Comunicate->ReadMotionMonitorData = 1u;
			break;

		case CMD_ACK_FAULT:
			USB_SendAckPacket(ACK, command, payload, payload_length);
			USB_HandleFaultAck();
			break;

		case CMD_READ_DRIVER:
			USB_SendAckPacket(ACK, command, payload, payload_length);
			USB_HandleReadDriver(USB_Comunicate, payload, payload_length);
			break;

		case CMD_READ_MOTOR:
			USB_SendAckPacket(ACK, command, payload, payload_length);
			USB_HandleReadMotor(USB_Comunicate, payload, payload_length);
			break;

		case CMD_WRITE_DRIVER:
			USB_SendAckPacket(ACK, command, payload, payload_length);
			USB_HandleWriteParameterPairs(DriverParameter, payload, payload_length, DRIVER_PARAMETER_COUNT);
			UpdateDriverParameter(DriverParameter);
			break;

		case CMD_WRITE_MOTOR:
			USB_SendAckPacket(ACK, command, payload, payload_length);
			USB_HandleWriteParameterPairs(MotorParameter, payload, payload_length, 32u);
			UpdateMotorParameter(MotorParameter);
			break;

		default:
			USB_SendAckPacket(ACK_ERROR, command, payload, payload_length);
			break;
	}
}

static uint8_t USB_TryAssembleFrame(uint8_t *frame, uint16_t *frame_length)
{
	uint8_t byte;

	while (USB_RingPop(&byte) == USB_COMM_OK)
	{
		if (s_parser.receiving == 0u)
		{
			if (byte == STX)
			{
				USB_ResetParser();
				s_parser.receiving = 1u;
				s_parser.buffer[s_parser.index++] = byte;
			}
			continue;
		}

		if (s_parser.index >= RX_FRAME_BUFFER_SIZE)
		{
			USB_ResetParser();
			continue;
		}

		s_parser.buffer[s_parser.index++] = byte;

		if (s_parser.index == 3u)
		{
			uint8_t size = s_parser.buffer[2];
			if ((size == 0u) || (size > 250u))
			{
				USB_ResetParser();
				continue;
			}
			s_parser.expected_length = (uint16_t)(size + 5u);
		}

		if ((s_parser.expected_length > 0u) && (s_parser.index == s_parser.expected_length))
		{
			if (s_parser.buffer[s_parser.expected_length - 1u] == ETX)
			{
				memcpy(frame, s_parser.buffer, s_parser.expected_length);
				*frame_length = s_parser.expected_length;
				USB_ResetParser();
				return USB_COMM_OK;
			}

			USB_ResetParser();
		}
	}

	return USB_COMM_FAIL;
}

void USB_ReceiveData(uint8_t *data, uint32_t SizeOfData)
{
	uint32_t index;

	for (index = 0u; index < SizeOfData; index++)
	{
		(void)USB_RingPush(data[index]);
	}
}

void USB_ProcessData(USB_Comunication_t *USB_Comunicate)
{
	uint8_t frame[RX_FRAME_BUFFER_SIZE];
	uint16_t frame_length;

	while (USB_TryAssembleFrame(frame, &frame_length) == USB_COMM_OK)
	{
		uint8_t code = frame[1];
		uint8_t size = frame[2];
		uint8_t command = frame[3];
		uint8_t payload_length;
		uint8_t crc;

		if (code != SYN)
		{
			continue;
		}

		payload_length = (uint8_t)(size - 1u);
		crc = frame[3u + size];

		if (CalcCRC(code, size, &frame[3]) != crc)
		{
			USB_SendAckPacket(ACK_ERROR, command, &frame[4], payload_length);
			continue;
		}

		USB_DispatchCommand(USB_Comunicate, command, &frame[4], payload_length);
	}
}

int16_t USB_TransmitData(USB_Comunication_t *USB_Comunicate)
{
	if (USB_Comunicate->SendError && (FaultCode != NO_ERROR))
	{
		USB_SendErrorPacket(USB_Comunicate);
	}

	if ((USB_Comunicate->ReadDriverParameter == 1u) && (USB_Comunicate->PriorityFlag == 2u))
	{
		if (USB_Comunicate->Counter < USB_Comunicate->TotalLength)
		{
			USB_SendParameterChunk(CMD_READ_DRIVER_DATA, DriverParameter, USB_Comunicate);
		}
		else
		{
			USB_Comunicate->ReadDriverParameter = 0u;
			USB_Comunicate->Counter = 0u;
			USB_Comunicate->TotalLength = 0u;
			USB_Comunicate->PriorityFlag = 0u;
		}
	}

	if ((USB_Comunicate->ReadMotorParameter == 1u) && (USB_Comunicate->PriorityFlag == 2u))
	{
		if (USB_Comunicate->Counter < USB_Comunicate->TotalLength)
		{
			USB_SendParameterChunk(CMD_READ_MOTOR_DATA, MotorParameter, USB_Comunicate);
		}
		else
		{
			USB_Comunicate->ReadMotorParameter = 0u;
			USB_Comunicate->Counter = 0u;
			USB_Comunicate->TotalLength = 0u;
			USB_Comunicate->PriorityFlag = 0u;
		}
	}

	if ((USB_Comunicate->ReadTraceData == 1u) && (USB_Comunicate->PriorityFlag == 1u))
	{
		USB_SendTraceChunk(USB_Comunicate);
	}

	if ((gMotorAutoTune.tuning_data_ready != 0u) &&
		(gMotorAutoTune.chart_transfer_active != 0u))
	{
		USB_SendAutoTuneChunk(USB_Comunicate);
	}

	if (USB_Comunicate->ReadFocDebugLog == 1u)
	{
		USB_SendFocDebugPacket(USB_Comunicate);
	}

	if (USB_Comunicate->ReadMotionMonitorData == 1u)
	{
		USB_SendMonitorPacket(USB_Comunicate);
	}

	return 0;
}

