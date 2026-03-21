#include "USBComunication.h"

#include <string.h>

#include "Parameter.h"
#include "StateMachine.h"
#include "define.h"
#include "pwm.h"
#include "usbd_cdc_if.h"

#define RX_BYTE_BUFFER_SIZE 512u
#define RX_FRAME_BUFFER_SIZE 260u
#define TX_FRAME_BUFFER_SIZE 256u
#define USB_COMM_OK 0u
#define USB_COMM_FAIL 1u

static const uint8_t STX = 0x02u;
static const uint8_t ACK = 0xF0u;
static const uint8_t ACK_ERROR = 0xFFu;
static const uint8_t SYN = 0x16u;
static const uint8_t ETX = 0x03u;

static volatile uint16_t s_rx_head = 0u;
static volatile uint16_t s_rx_tail = 0u;
static uint8_t s_rx_buffer[RX_BYTE_BUFFER_SIZE];

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
extern float DriverParameter[16];
extern float MotorParameter[32];
extern float gIdRefA;
extern float gIqRefA;
extern volatile float gTargetSpeedRpm;
extern volatile uint8_t gRunMode;
extern volatile float gVfFrequencyHz;
extern volatile float gVfVoltageV;
extern uint16_t FaultCode;
extern float gTracePosError;
extern float RecordTable1[TRACE_DATA_LENGTH * 4u];
extern TraceData Trace_Data;
extern IdSquareTuning_t IdSquareTuning;

static void USB_StartServoSequence(void);

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

static void USB_SendMonitorPacket(USB_Comunication_t *USB_Comunicate)
{
	uint8_t offset = 0u;
	float vdc_scaled = Parameter.fVdc * 0.1f;
	float speed_error = gTargetSpeedRpm - Parameter.fActSpeed;
	float cmd_position = 0.0f;
	float position_error = 0.0f;
	int16_t motor_power = (int16_t)(Parameter.fVdc * gIqRefA);
	uint8_t enable_run = (uint8_t)((StateMachine.bState == RUN) ? 1u : 0u);
	uint16_t fault = FaultCode;
	uint8_t run_mode = (uint8_t)gRunMode;
	float id_ref = gIdRefA;

	memcpy(&USB_Comunicate->TransmitData[offset], &enable_run, sizeof(enable_run));
	offset = (uint8_t)(offset + sizeof(enable_run));
	memcpy(&USB_Comunicate->TransmitData[offset], &vdc_scaled, sizeof(vdc_scaled));
	offset = (uint8_t)(offset + sizeof(vdc_scaled));
	memcpy(&USB_Comunicate->TransmitData[offset], &Parameter.fTemparature, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gTargetSpeedRpm, sizeof(float));
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

	SendData(CMD_MONITOR_DATA, offset, USB_Comunicate->TransmitData);
	USB_Comunicate->ReadMotionMonitorData = 0u;
}

static void USB_SendErrorPacket(USB_Comunication_t *USB_Comunicate)
{
	uint8_t offset = 0u;
	int16_t motor_power = (int16_t)(Parameter.fVdc * gIqRefA);

	memcpy(&USB_Comunicate->TransmitData[offset], &FaultCode, sizeof(FaultCode));
	offset = (uint8_t)(offset + sizeof(FaultCode));
	memcpy(&USB_Comunicate->TransmitData[offset], &Parameter.fVdc, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], &Parameter.fTemparature, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], (const void *)&gTargetSpeedRpm, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], &Parameter.fActSpeed, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	{
		float speed_error = gTargetSpeedRpm - Parameter.fActSpeed;
		memcpy(&USB_Comunicate->TransmitData[offset], &speed_error, sizeof(float));
		offset = (uint8_t)(offset + sizeof(float));
	}
	{
		float cmd_position = 0.0f;
		memcpy(&USB_Comunicate->TransmitData[offset], &cmd_position, sizeof(float));
		offset = (uint8_t)(offset + sizeof(float));
	}
	memcpy(&USB_Comunicate->TransmitData[offset], &Parameter.fPosition, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	{
		float position_error = 0.0f;
		memcpy(&USB_Comunicate->TransmitData[offset], &position_error, sizeof(float));
		offset = (uint8_t)(offset + sizeof(float));
	}
	memcpy(&USB_Comunicate->TransmitData[offset], &gIqRefA, sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], &motor_power, sizeof(motor_power));
	offset = (uint8_t)(offset + sizeof(motor_power));
	memcpy(&USB_Comunicate->TransmitData[offset], &Parameter.fIabc[0], sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], &Parameter.fIabc[1], sizeof(float));
	offset = (uint8_t)(offset + sizeof(float));
	memcpy(&USB_Comunicate->TransmitData[offset], &Parameter.fIabc[2], sizeof(float));
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
	uint8_t total_length = (payload_length > 0u) ? payload[0] : 16u;
	if (total_length > 16u)
	{
		total_length = 16u;
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

	if (payload_length < 16u)
	{
		return;
	}

	memcpy(&amplitude, &payload[0], sizeof(float));
	memcpy(&frequency, &payload[4], sizeof(float));
	memcpy(&current_kp, &payload[8], sizeof(float));
	memcpy(&current_ki, &payload[12], sizeof(float));

	IdSquareTuning.fAmplitudeCmd = amplitude;
	IdSquareTuning.fFrequencyCmd = frequency;
	IdSquareTuning.fPhase = 0.0f;

	MotorParameter[MOTOR_CURRENT_P_GAIN] = current_kp;
	MotorParameter[MOTOR_CURRENT_I_GAIN] = current_ki;
	UpdateMotorParameter(MotorParameter);
}

static void USB_HandleIdSquareTuningStart(void)
{
	if ((StateMachine.bState != IDLE) && (StateMachine.bState != STOP))
	{
		IdSquareTuning.Enable = 0u;
		IdSquareTuning.fPhase = 0.0f;
		return;
	}

	gRunMode = RUN_MODE_FOC;
	gVfFrequencyHz = 0.0f;
	gVfVoltageV = 0.0f;
	gTargetSpeedRpm = 0.0f;
	gIdRefA = 0.0f;
	gIqRefA = 0.0f;
	IdSquareTuning.Enable = 1u;
	IdSquareTuning.fPhase = 0.0f;
	IdSquareTuning.fVoltageLimitApplied = 0.0f;
	USB_StartServoSequence();
	if (Trace_Data.Enable != 0u)
	{
		USB_ResetTraceCaptureState();
	}
}

static void USB_HandleIdSquareTuningStop(void)
{
	IdSquareTuning.Enable = 0u;
	IdSquareTuning.fPhase = 0.0f;
	IdSquareTuning.fVoltageLimitApplied = 0.0f;
	gTargetSpeedRpm = 0.0f;
	gIdRefA = 0.0f;
	gIqRefA = 0.0f;
	STM_NextState(&StateMachine, STOP);
}

static void USB_StartServoSequence(void)
{
	Reset_CurrentSensor(&Current_Sensor);
	FaultCode = NO_ERROR;
	USB_Comm.SendError = false;
	STM_NextState(&StateMachine, OFFSET_CALIB);
}

static void USB_HandleSpeedCommand(const uint8_t *payload, uint8_t payload_length)
{
	gRunMode = RUN_MODE_FOC;
	gVfFrequencyHz = 0.0f;
	gVfVoltageV = 0.0f;
	IdSquareTuning.Enable = 0u;
	IdSquareTuning.fPhase = 0.0f;
	IdSquareTuning.fVoltageLimitApplied = 0.0f;
	ResetControl_V_over_F();

	if (payload_length >= sizeof(float))
	{
		memcpy((void *)&gTargetSpeedRpm, payload, sizeof(float));
	}

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

	if (requested_frequency < 0.0f)
	{
		requested_frequency = 0.0f;
	}
	if (requested_frequency > 50.0f)
	{
		requested_frequency = 50.0f;
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
	gIdRefA = 0.0f;
	gIqRefA = 0.0f;
	IdSquareTuning.Enable = 0u;
	IdSquareTuning.fPhase = 0.0f;
	IdSquareTuning.fVoltageLimitApplied = 0.0f;
	gRunMode = RUN_MODE_OPEN_LOOP_VF;
	ResetControl_V_over_F();
}

static void USB_HandleFaultAck(void)
{
	FaultCode = NO_ERROR;
	USB_Comm.SendError = false;
	gRunMode = RUN_MODE_FOC;
	gVfFrequencyHz = 0.0f;
	gVfVoltageV = 0.0f;
	IdSquareTuning.Enable = 0u;
	IdSquareTuning.fPhase = 0.0f;
	IdSquareTuning.fVoltageLimitApplied = 0.0f;
	ResetControl_V_over_F();
	STM_FaultProcessing(&StateMachine, 0u, 0u);
	STM_NextState(&StateMachine, IDLE);
}

static void USB_HandleServoOn(void)
{
	gRunMode = RUN_MODE_FOC;
	gVfFrequencyHz = 0.0f;
	gVfVoltageV = 0.0f;
	IdSquareTuning.Enable = 0u;
	IdSquareTuning.fPhase = 0.0f;
	IdSquareTuning.fVoltageLimitApplied = 0.0f;
	ResetControl_V_over_F();
	USB_StartServoSequence();
}

static void USB_HandleServoOff(void)
{
	gRunMode = RUN_MODE_FOC;
	gVfFrequencyHz = 0.0f;
	gVfVoltageV = 0.0f;
	ResetControl_V_over_F();
	IdSquareTuning.Enable = 0u;
	IdSquareTuning.fPhase = 0.0f;
	IdSquareTuning.fVoltageLimitApplied = 0.0f;
	gTargetSpeedRpm = 0.0f;
	gIdRefA = 0.0f;
	gIqRefA = 0.0f;
	STM_NextState(&StateMachine, STOP);
}

static void USB_HandleOpenLoopStop(void)
{
	gRunMode = RUN_MODE_FOC;
	gVfFrequencyHz = 0.0f;
	gVfVoltageV = 0.0f;
	ResetControl_V_over_F();
	IdSquareTuning.Enable = 0u;
	IdSquareTuning.fPhase = 0.0f;
	IdSquareTuning.fVoltageLimitApplied = 0.0f;
	gTargetSpeedRpm = 0.0f;
	gIdRefA = 0.0f;
	gIqRefA = 0.0f;
	STM_NextState(&StateMachine, STOP);
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
				USB_StartServoSequence();
			}
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
				USB_StartServoSequence();
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
			USB_HandleWriteParameterPairs(DriverParameter, payload, payload_length, 16u);
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

	if (USB_Comunicate->ReadMotionMonitorData == 1u)
	{
		USB_SendMonitorPacket(USB_Comunicate);
	}

	return 0;
}

