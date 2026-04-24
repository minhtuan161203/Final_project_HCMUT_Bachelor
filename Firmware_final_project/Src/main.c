
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2026 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "define.h"
#include "PIDcontrol.h"
#include "StateMachine.h"
#include "USBComunication.h"
#include "vector_transfs.h"
#include <math.h>
#include "pwm.h"
#include <stdint.h>
#include <string.h>
#include "Parameter.h"
#include "motor_autotune.h"
#if defined(__CC_ARM) || defined(__ARMCC_VERSION)
#include <stdio.h>
#endif
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim8;

SRAM_HandleTypeDef hsram1;
SRAM_HandleTypeDef hsram2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

float DriverParameter[DRIVER_PARAMETER_COUNT];
float MotorParameter[32];
uint16_t FaultCode = NO_ERROR;
float globalcontrolthetatune = -0.05f;
uint8_t system_on = 0;
volatile float gTargetSpeedRpm = 0.0f;
volatile uint8_t gRunMode = RUN_MODE_FOC;
volatile float gVfFrequencyHz = 0.0f;
volatile float gVfVoltageV = 0.0f;
volatile float gOpenLoopFrequencyLimitHz = DEFAULT_MOTOR_RATED_ELECTRICAL_FREQUENCY_HZ;
volatile float gTargetPositionCounts = 0.0f;
volatile float gCommandedSpeedRpm = 0.0f;
float gIdRefA = 0.0f;
float gIqRefA = 0.0f;
volatile uint16_t gRawCurrentU = 0u;
volatile uint16_t gRawCurrentV = 0u;
volatile uint16_t gRawVdc = 0u;
volatile uint16_t gRawTemp = 0u;
volatile uint16_t gDebugFaultSnapshot = 0u;
volatile float gDebugSpeedRawRpm = 0.0f;
volatile float gDebugSpeedRawRpmAvg = 0.0f;
volatile float gDebugObservedElectricalHz = 0.0f;
volatile float gDebugObservedElectricalHzAvg = 0.0f;
volatile float gDebugOpenLoopElectricalHzCmd = 0.0f;
volatile float gDebugOpenLoopSyncRpmCmd = 0.0f;
volatile float gDebugExpectedDeltaPosSync = 0.0f;
volatile float gDebugDeltaPosAvg = 0.0f;
volatile float gDebugEncoderTurns = 0.0f;
volatile float gDebugMechanicalAngleRad = 0.0f;
volatile float gDebugElectricalAngleRad = 0.0f;
volatile uint32_t gDebugIsrDeltaCycles = 0u;
volatile float gDebugIsrPeriodUs = 0.0f;
volatile float gDebugIsrFrequencyHz = 0.0f;
volatile uint8_t gIsrMeasureOnlyMode = 0u;
volatile uint32_t gIsrMeasureEdgeCount = 0u;
volatile float gIsrMeasureEdgeFrequencyHz = 0.0f;
volatile uint8_t gControlTimingMode = USER_DEFAULT_CONTROL_TIMING_MODE;
volatile float gEffectiveCurrentLoopFrequencyHz = USER_SELECTED_ISR_FREQUENCY;
volatile float gEffectiveSpeedLoopFrequencyHz = USER_EFFECTIVE_SPEED_LOOP_FREQUENCY;
float gTracePosError = 0.0f;
volatile uint8_t gEncoderAlignmentPolicy = ENCODER_ALIGNMENT_POLICY_POWER_ON;
volatile uint8_t gEncoderAlignmentStatus = ENCODER_ALIGNMENT_STATUS_IDLE;
volatile uint8_t gEncoderAlignmentNeedsFlashSave = 0u;
volatile int32_t gEncoderAlignmentLastCapturedOffset = 0;
volatile uint8_t gEncoderAlignmentRequested = 0u;
volatile uint8_t gEncoderAlignmentContinueToRun = 0u;
volatile uint8_t gEncoderAlignmentResumeRunMode = RUN_MODE_FOC;
volatile uint8_t gServoArmOnlyRequested = 0u;
volatile uint8_t gFocElectricalAngleTestMode = ID_SQUARE_ANGLE_TEST_NONE;
volatile uint8_t gFocCurrentUvSwapTest = 0u;
volatile uint8_t gFocCurrentPolarityInvertTest = 0u;
volatile uint8_t gFpgaEncoderParserConfigured = 0u;
volatile uint8_t gFpgaDoneLatched = 0u;
volatile uint32_t gFpgaDoneTickMs = 0u;
volatile uint8_t gFocRotatingThetaTestRunning = 0u;
volatile float gFocRotatingThetaDebugThetaDeg = 0.0f;
volatile int32_t gFocRotatingThetaDebugDeltaPos = 0;
volatile uint8_t gFocRotatingThetaVoltageTestRunning = 0u;
volatile float gFocRotatingThetaVoltageDebugThetaDeg = 0.0f;
volatile int32_t gFocRotatingThetaVoltageDebugDeltaPos = 0;
volatile uint8_t gFocCurrentFeedbackMapTestRunning = 0u;
volatile float gFaultPhaseU = 0.0f;
volatile float gFaultPhaseV = 0.0f;
volatile float gFaultPhaseW = 0.0f;
float RecordTable1[TRACE_DATA_LENGTH * 4u];
TraceData Trace_Data;
IdSquareTuning_t IdSquareTuning;
MotorAutoTune_t gMotorAutoTune;

Parameterhandle_t Parameter;
CurrentSensor_t Current_Sensor;
StateMachine_t StateMachine;
USB_Comunication_t USB_Comm;
tFRClarke gClarke = FR_CLARKE_DEFAULTS;
tFPark gPark = F_PARK_DEFAULTS;
tIPark gInvPark = I_PARK_DEFAULTS;
tIFClarke gInvClarke = IF_CLARKE_DEFAULTS;
tPI gIdPi = PI_DEFAULTS;
tPI gIqPi = PI_DEFAULTS;
tPI gSpeedPi = PI_DEFAULTS;
tPI gPositionPi = PI_DEFAULTS;
static uint8_t gPwmEnabled = 0u;
static uint8_t gSpeedLoopDivider = 0u;

volatile uint8_t data_received_global;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM1_Init(void);
static void MX_FSMC_Init(void);
static void MX_ADC1_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static float ClampFloat(float value, float lower, float upper);
static float WrapAngle(float angle);
static void SetPwmEnabled(uint8_t enable);
static void ResetControlLoops(void);
static void LoadDefaultParameters(void);
static void ReadFastProtectionFeedback(void);
static void UpdateMeasuredSpeedAndTheta(void);
static void LimitDqVoltageVector(float *vd, float *vq, float limit);
static float GetOpenLoopVoltageLimit(void);
static void ApplyOpenLoopVfCommand(float requested_frequency, float requested_voltage);
static void RunOpenLoopVf(void);
static void ReportFault(uint16_t fault);
static void RunFocLoop(void);
static void UpdateMeasuredCurrentsForTheta(float electrical_theta, float phase_u, float phase_v);
static void ApplyPhaseCurrentFeedbackMapping(float *phase_u, float *phase_v);
static void ApplyVoltageVectorForTheta(float electrical_theta, float vd, float vq);
static void RunCurrentLoopForTheta(
	float electrical_theta,
	float id_ref,
	float iq_ref,
	float voltage_limit,
	uint8_t isolate_q_axis);
static uint8_t ShouldHoldCurrentLoopAtZero(float current_ref, float measured_current);
static void RunMotorAutoTuneLoop(void);
static float CalcIdSquareTuningVoltageLimit(void);
static float CalcIdSquareTuningReference(void);
static float CalcIdSquareTuningAlignmentCurrent(void);
static int32_t CalcAlignedEncoderOffset(float encoder_resolution);
static float *ResolveTraceChannelPointer(uint8_t channel_code);
static void UpdateTraceCapture(void);
static void ResetDebugAveraging(void);
static void InitIsrDebugCounter(void);
static void UpdateIsrDebugPeriod(void);
static void UpdateIsrMeasureOnlyFrequency(void);
static float GetEffectiveCurrentLoopFrequency(void);
static float GetEffectiveSpeedLoopFrequency(void);
static uint8_t GetActiveFocControlMode(void);
static float GetConfiguredPositionGain(void);
static float GetConfiguredPositionIntegralGain(void);
static float GetConfiguredPositionVffGain(void);
static float GetConfiguredPositionVffFilterHz(void);
static uint8_t GetConfiguredPositionTrackingMode(void);
static float GetConfiguredSpeedLimitRpm(void);
static float GetConfiguredSpeedIqLimitA(void);
static float ApplySpeedRampLimit(float current_command, float target_command, float max_speed_rpm, float dt_sec);
static float WrapPositionErrorCounts(float error_counts, float encoder_resolution);
static float GetPositionControlErrorCounts(float target_position_counts, float actual_position_counts, float encoder_resolution);
static float GetPositionLoopErrorDeadbandCounts(float encoder_resolution);
static float GetPositionLoopErrorReleaseDeadbandCounts(float encoder_resolution);
static float UpdatePositionSetpointVelocityRpm(float target_position_counts, float encoder_resolution, float dt_sec);
static uint8_t IsAbsoluteEncoderId(uint32_t encoder_id);
static void RefreshEncoderAlignmentPolicy(void);
static void ResetEncoderAlignmentAveraging(void);
static float GetAlignedSingleTurnPositionCounts(float encoder_resolution);
static int32_t CalcAlignedEncoderOffsetFromCounts(float single_turn_position, float encoder_resolution);
static void AccumulateEncoderAlignmentSample(float encoder_resolution);
static int32_t CalcAveragedAlignedEncoderOffset(float encoder_resolution);
static void RestoreIdSquareTuningOffsetGuard(void);
static float GetAngleTestModeOffsetRad(uint8_t electrical_angle_test_mode);
static float GetAngleTestModeOffsetDeg(uint8_t electrical_angle_test_mode);
static float GetRuntimeFocControlTheta(void);
static float GetIdSquareLockedControlTheta(void);
static float GetAutoTuneControlTheta(void);
static int32_t ElectricalAngleDegToEncoderCounts(float electrical_deg, float encoder_resolution, uint8_t pole_pairs);
static void ApplyEncoderOffsetElectricalDelta(float electrical_deg, float encoder_resolution);
static void FinalizeEncoderAlignment(uint8_t alignment_successful);
static void RunFocRotatingThetaTestLoop(void);
static void RunFocRotatingThetaVoltageTestLoop(void);
static void RunFocCurrentFeedbackMapTestLoop(void);
static void LoadDiagnosticCurrentPiGains(void);
static uint8_t LoadParametersFromFlashIfAvailable(void);
static const char *GetCurrentFeedbackMapCaseName(uint8_t index);
static void ServiceFpgaStartup(void);
void ApplyControlTimingMode(uint8_t mode);
void PrepareEncoderAlignment(uint8_t continue_to_run, uint8_t resume_run_mode);
uint8_t StartFocRotatingThetaTest(void);
uint8_t StartFocRotatingThetaVoltageTest(void);
uint8_t StartFocCurrentFeedbackMapTest(void);
void StopFocDiagnosticModes(void);
uint8_t SaveParametersToFlash(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define FOC_DEBUG_ENABLE 0u
#if (FOC_DEBUG_ENABLE != 0u)
#define FOC_DEBUG_PRINTF(...) printf(__VA_ARGS__)
#else
#define FOC_DEBUG_PRINTF(...) ((void)0)
#endif

#define ID_SQUARE_TUNING_MAX_RATED_CURRENT_RATIO 0.20f
#define ID_SQUARE_TUNING_MAX_OC_RATIO 0.20f
#define ID_SQUARE_TUNING_ALIGN_VOLTAGE_RATIO 0.20f
#define ID_SQUARE_TUNING_SQUARE_VOLTAGE_RATIO 0.35f
#define ID_SQUARE_TUNING_MAX_FREQUENCY_HZ 50.0f
#define ID_SQUARE_TUNING_MIN_VOLTAGE_LIMIT_V 1.0f
#define FOC_SPEED_LOOP_MAX_OC_RATIO 0.60f
#define FOC_SPEED_LOOP_MIN_IQ_LIMIT_A 0.20f
#define FOC_ZERO_CMD_REF_DEADBAND_A 0.01f
#define FOC_ZERO_CMD_MEAS_DEADBAND_A 0.08f
#define POSITION_LOOP_ERROR_DEADBAND_DEG 0.50f
#define POSITION_LOOP_ERROR_RELEASE_DEADBAND_DEG 0.80f
#define POSITION_LOOP_FF_DEADBAND_RPM 0.05f
#define POSITION_VFF_FILTER_DEFAULT_HZ 50.0f
#define FORCE_DRIVER_CURRENT_POLARITY_INVERT 0u
#define FORCE_DRIVER_CURRENT_UV_SWAP 1u
#define ID_SQUARE_TUNING_ALIGN_CURRENT_MIN_A 0.2f
#define ID_SQUARE_TUNING_ALIGN_CURRENT_RATIO 0.2f
#define ID_SQUARE_TUNING_ALIGN_TIME_S 2.0f
#define ENCODER_ALIGNMENT_AVG_SAMPLES 128u
#define DIRECTION_TEST_OPEN_LOOP_FREQ_HZ 5.0f
#define DIRECTION_TEST_OPEN_LOOP_VOLTAGE_V 2.0f
#define DIRECTION_TEST_IQ_RATIO 0.05f
#define DIRECTION_TEST_MIN_IQ_A 0.01f
#define DIRECTION_TEST_MAX_IQ_A 0.06f
#define DIRECTION_TEST_MOVE_TIME_S 0.18f
#define DIRECTION_TEST_SETTLE_TIME_S 0.08f
#define DIRECTION_TEST_VOLTAGE_LIMIT_RATIO 0.10f
#define DIRECTION_TEST_MIN_VOLTAGE_LIMIT_V 2.0f
#define DIRECTION_TEST_MAX_VOLTAGE_LIMIT_V 6.0f
#define ROTATING_THETA_TEST_ELECTRICAL_FREQ_HZ 1.0f
#define ROTATING_THETA_TEST_IQ_A 0.20f
#define ROTATING_THETA_TEST_FRAME_DEG (-DEFAULT_ELECTRICAL_ALIGNMENT_OFFSET_DEG)
#define DEFAULT_RUNTIME_FOC_FRAME_DEG (-DEFAULT_ELECTRICAL_ALIGNMENT_OFFSET_DEG)
#define ROTATING_THETA_TEST_VOLTAGE_LIMIT_RATIO 0.15f
#define ROTATING_THETA_TEST_MIN_VOLTAGE_LIMIT_V 2.0f
#define ROTATING_THETA_TEST_MAX_VOLTAGE_LIMIT_V 8.0f
#define ROTATING_THETA_TEST_LOG_TIME_S 0.25f
#define ROTATING_THETA_VOLTAGE_TEST_ELECTRICAL_FREQ_HZ 1.0f
#define ROTATING_THETA_VOLTAGE_TEST_VQ_V 2.0f
#define ROTATING_THETA_VOLTAGE_TEST_FRAME_DEG (-DEFAULT_ELECTRICAL_ALIGNMENT_OFFSET_DEG)
#define ROTATING_THETA_VOLTAGE_TEST_LOG_TIME_S 0.25f
#define CURRENT_FEEDBACK_MAP_TEST_CASE_COUNT 4u
#define CURRENT_FEEDBACK_MAP_TEST_CASE_TIME_S 1.0f
#define FOC_DIAGNOSTIC_CURRENT_KP_SCALE 1.0f
#define FOC_DIAGNOSTIC_CURRENT_KI_SCALE 1.0f
#define SPEED_ESTIMATE_LPF_ALPHA 0.1f
#define DEBUG_AVG_SAMPLES 256u
#define MOTOR_PARAMETER_COUNT 32u

static float sDebugDeltaPosAccum = 0.0f;
static float sDebugSpeedRawAccum = 0.0f;
static float sDebugObservedElecHzAccum = 0.0f;
static uint16_t sDebugAvgCounter = 0u;
static uint32_t sLastIsrCycleCount = 0u;
static uint8_t sIsrDebugCounterReady = 0u;
static uint32_t sIsrMeasurePrevCount = 0u;
static uint32_t sIsrMeasurePrevTickMs = 0u;
static uint8_t sIsrMeasurePrevMode = 0u;
static float sEncoderAlignSinAccum = 0.0f;
static float sEncoderAlignCosAccum = 0.0f;
static uint16_t sEncoderAlignSampleCount = 0u;
static uint8_t sIdSquareTuningOffsetGuardActive = 0u;
static int32_t sIdSquareTuningSavedOffset = 0;
static float sFocRotatingThetaCommand = 0.0f;
static float sFocRotatingThetaStartPosition = 0.0f;
static uint16_t sFocRotatingThetaLogCounter = 0u;
static float sFocRotatingThetaVoltageCommand = 0.0f;
static float sFocRotatingThetaVoltageStartPosition = 0.0f;
static uint16_t sFocRotatingThetaVoltageLogCounter = 0u;
static float sPositionSetpointPrevCounts = 0.0f;
static float sPositionSetpointVelocityCountsPerSec = 0.0f;
static uint8_t sPositionDeadbandHoldActive = 0u;
static const uint8_t sFocCurrentFeedbackMapSwapCases[CURRENT_FEEDBACK_MAP_TEST_CASE_COUNT] = {0u, 0u, 1u, 1u};
static const uint8_t sFocCurrentFeedbackMapInvertCases[CURRENT_FEEDBACK_MAP_TEST_CASE_COUNT] = {0u, 1u, 0u, 1u};
static uint8_t sFocCurrentFeedbackMapCaseIndex = 0u;
static uint16_t sFocCurrentFeedbackMapCounter = 0u;
static float sFocCurrentFeedbackMapCommand = 0.0f;
static float sFocCurrentFeedbackMapStartPosition = 0.0f;
static float sFocCurrentFeedbackMapIdAbsAccum = 0.0f;
static float sFocCurrentFeedbackMapIqAccum = 0.0f;
static float sFocCurrentFeedbackMapIqErrAbsAccum = 0.0f;
static float sFocCurrentFeedbackMapSpeedAbsAccum = 0.0f;
static uint32_t sFocCurrentFeedbackMapSampleCount = 0u;
static float sFocCurrentFeedbackMapIdAbsAvg[CURRENT_FEEDBACK_MAP_TEST_CASE_COUNT] = {0.0f};
static float sFocCurrentFeedbackMapIqAvg[CURRENT_FEEDBACK_MAP_TEST_CASE_COUNT] = {0.0f};
static float sFocCurrentFeedbackMapIqErrAbsAvg[CURRENT_FEEDBACK_MAP_TEST_CASE_COUNT] = {0.0f};
static float sFocCurrentFeedbackMapSpeedAbsAvg[CURRENT_FEEDBACK_MAP_TEST_CASE_COUNT] = {0.0f};
static int32_t sFocCurrentFeedbackMapDeltaPos[CURRENT_FEEDBACK_MAP_TEST_CASE_COUNT] = {0};

static const char *GetCurrentFeedbackMapCaseName(uint8_t index)
{
	switch (index)
	{
		case 0u:
			return "normal";
		case 1u:
			return "invert";
		case 2u:
			return "swap";
		case 3u:
			return "swap+invert";
		default:
			return "unknown";
	}
}

static void ServiceFpgaStartup(void)
{
	uint32_t now_tick_ms;
	GPIO_PinState fpga_done_pin;

	now_tick_ms = HAL_GetTick();
	fpga_done_pin = HAL_GPIO_ReadPin(iFPGA_DONE_GPIO_Port, iFPGA_DONE_Pin);

	if (fpga_done_pin == GPIO_PIN_SET)
	{
		if (gFpgaDoneLatched == 0u)
		{
			gFpgaDoneLatched = 1u;
			gFpgaDoneTickMs = now_tick_ms;
		}

		/* After cold power-up, the MCU can reach REG_ENCODER_ID writes before the
		 * FPGA encoder parser is ready. Re-apply the parser selection only after
		 * FPGA_DONE is stably high so encoder updates start without a debugger rerun. */
		if ((gFpgaEncoderParserConfigured == 0u) &&
			((now_tick_ms - gFpgaDoneTickMs) >= 10u))
		{
			*(__IO uint16_t*)(REG_ENCODER_ID) = (uint16_t)MotorParameter[MOTOR_ENCODER_ID];
			gFpgaEncoderParserConfigured = 1u;
		}
	}
	else
	{
		gFpgaDoneLatched = 0u;
		gFpgaDoneTickMs = now_tick_ms;
		gFpgaEncoderParserConfigured = 0u;
	}
}

static float ClampFloat(float value, float lower, float upper)
{
	if (value > upper)
	{
		return upper;
	}
	if (value < lower)
	{
		return lower;
	}
	return value;
}

static float WrapAngle(float angle)
{
    angle = fmodf(angle, 2.0f * PI);
    if (angle < 0.0f)
    {
        angle += (2.0f * PI);
    }
    return angle;
}

static float GetAngleTestModeOffsetRad(uint8_t electrical_angle_test_mode)
{
	switch (electrical_angle_test_mode)
	{
		case ID_SQUARE_ANGLE_TEST_PLUS_90:
			return 0.5f * PI;
		case ID_SQUARE_ANGLE_TEST_MINUS_90:
			return -0.5f * PI;
		case ID_SQUARE_ANGLE_TEST_PLUS_180:
			return PI;
		case ID_SQUARE_ANGLE_TEST_NONE:
		default:
			return 0.0f;
	}
}

static float GetAngleTestModeOffsetDeg(uint8_t electrical_angle_test_mode)
{
	switch (electrical_angle_test_mode)
	{
		case ID_SQUARE_ANGLE_TEST_PLUS_90:
			return 90.0f;
		case ID_SQUARE_ANGLE_TEST_MINUS_90:
			return -90.0f;
		case ID_SQUARE_ANGLE_TEST_PLUS_180:
			return 180.0f;
		case ID_SQUARE_ANGLE_TEST_NONE:
		default:
			return 0.0f;
	}
}

static float GetRuntimeFocControlTheta(void)
{
	return WrapAngle(
		Parameter.fTheta + ((DEFAULT_RUNTIME_FOC_FRAME_DEG * PI) / 180.0f));
}

static float GetIdSquareLockedControlTheta(void)
{
	return WrapAngle(-90.0f + globalcontrolthetatune);
}

static float GetAutoTuneControlTheta(void)
{
	if ((gMotorAutoTune.state == MOTOR_AUTOTUNE_STATE_RS) ||
		(gMotorAutoTune.state == MOTOR_AUTOTUNE_STATE_LS))
	{
		/* Keep Rs/Ls injection on the same locked d-axis frame that already
		   works for Id tuning so Vd does not leak into torque-producing Vq. */
		return GetIdSquareLockedControlTheta();
	}

	return GetRuntimeFocControlTheta();
}

static int32_t ElectricalAngleDegToEncoderCounts(float electrical_deg, float encoder_resolution, uint8_t pole_pairs)
{
	if ((encoder_resolution <= 1.0f) || (pole_pairs == 0u))
	{
		return 0;
	}
	return (int32_t)lroundf(
		(electrical_deg * encoder_resolution) /
		(360.0f * (float)pole_pairs));
}

static void ApplyEncoderOffsetElectricalDelta(float electrical_deg, float encoder_resolution)
{
	float wrapped_offset;
	int32_t offset_delta_counts;

	if ((encoder_resolution <= 1.0f) || (Parameter.u8PolePair == 0u))
	{
		return;
	}

	offset_delta_counts = ElectricalAngleDegToEncoderCounts(
		electrical_deg,
		encoder_resolution,
		Parameter.u8PolePair);
	wrapped_offset = (float)Parameter.Offset_Enc + (float)offset_delta_counts;
	wrapped_offset = fmodf(wrapped_offset, encoder_resolution);
	if (wrapped_offset < 0.0f)
	{
		wrapped_offset += encoder_resolution;
	}
	if (wrapped_offset > (0.5f * encoder_resolution))
	{
		wrapped_offset -= encoder_resolution;
	}

	Parameter.Offset_Enc = (int32_t)lroundf(wrapped_offset);
	MotorParameter[MOTOR_HALL_OFFSET] = (float)Parameter.Offset_Enc;
	UpdateMotorParameter(MotorParameter);
	gEncoderAlignmentLastCapturedOffset = Parameter.Offset_Enc;
	gEncoderAlignmentStatus = ENCODER_ALIGNMENT_STATUS_DONE;
	if (gEncoderAlignmentPolicy == ENCODER_ALIGNMENT_POLICY_MANUAL_SAVE)
	{
		gEncoderAlignmentNeedsFlashSave = 1u;
	}
	else
	{
		gEncoderAlignmentNeedsFlashSave = 0u;
	}
}

static void RestoreIdSquareTuningOffsetGuard(void)
{
	if (sIdSquareTuningOffsetGuardActive == 0u)
	{
		return;
	}

	Parameter.Offset_Enc = sIdSquareTuningSavedOffset;
	MotorParameter[MOTOR_HALL_OFFSET] = (float)sIdSquareTuningSavedOffset;
	gEncoderAlignmentLastCapturedOffset = sIdSquareTuningSavedOffset;
	FOC_DEBUG_PRINTF(
		"[IDTUNE] guard restore offset=%ld\r\n",
		(long)sIdSquareTuningSavedOffset);
	sIdSquareTuningOffsetGuardActive = 0u;
}

static void LoadDiagnosticCurrentPiGains(void)
{
	float diag_kp = MotorParameter[MOTOR_CURRENT_P_GAIN] * FOC_DIAGNOSTIC_CURRENT_KP_SCALE;
	float diag_ki = MotorParameter[MOTOR_CURRENT_I_GAIN] * FOC_DIAGNOSTIC_CURRENT_KI_SCALE;

	if (diag_kp < 0.0f)
	{
		diag_kp = 0.0f;
	}
	if (diag_ki < 0.0f)
	{
		diag_ki = 0.0f;
	}

	gIdPi.fKp = diag_kp;
	gIqPi.fKp = diag_kp;
	gIdPi.fKi = diag_ki;
	gIqPi.fKi = diag_ki;
}

static uint8_t ShouldHoldCurrentLoopAtZero(float current_ref, float measured_current)
{
	return ((fabsf(current_ref) <= FOC_ZERO_CMD_REF_DEADBAND_A) &&
		(fabsf(measured_current) <= FOC_ZERO_CMD_MEAS_DEADBAND_A)) ? 1u : 0u;
}

static void ResetDebugAveraging(void)
{
	sDebugDeltaPosAccum = 0.0f;
	sDebugSpeedRawAccum = 0.0f;
	sDebugObservedElecHzAccum = 0.0f;
	sDebugAvgCounter = 0u;
	gDebugDeltaPosAvg = 0.0f;
	gDebugSpeedRawRpmAvg = 0.0f;
	gDebugObservedElectricalHzAvg = 0.0f;
}

static uint8_t IsAbsoluteEncoderId(uint32_t encoder_id)
{
	switch (encoder_id)
	{
		case TAMAGAWA_SERIAL_ABS_SINGLE_TURN:
		case TAMAGAWA_SERIAL_ABS_MULTI_TURN:
		case TAMAGAWA_SERIAL_ABS_MULTI_TURN_23BIT:
		case PANASONIC_MINAS_A5_SERIAL_ABS:
		case PANASONIC_MINAS_A6_SERIAL_ABS:
		case YASKAWA_SIGMA_5_ABS_SINGLE_TURN:
			return 1u;
		default:
			return 0u;
	}
}

static void RefreshEncoderAlignmentPolicy(void)
{
	gEncoderAlignmentPolicy = (IsAbsoluteEncoderId((uint32_t)MotorParameter[MOTOR_ENCODER_ID]) != 0u) ?
		ENCODER_ALIGNMENT_POLICY_MANUAL_SAVE : ENCODER_ALIGNMENT_POLICY_POWER_ON;
	if (gEncoderAlignmentPolicy == ENCODER_ALIGNMENT_POLICY_POWER_ON)
	{
		gEncoderAlignmentNeedsFlashSave = 0u;
	}
}

void PrepareEncoderAlignment(uint8_t continue_to_run, uint8_t resume_run_mode)
{
	float current_kp = MotorParameter[MOTOR_CURRENT_P_GAIN];
	float current_ki = MotorParameter[MOTOR_CURRENT_I_GAIN];
	float rated_current = MotorParameter[MOTOR_RATED_CURRENT_RMS];
	float align_current;

	if (current_kp <= 0.0f)
	{
		current_kp = 1.0f;
	}
	if (current_ki < 0.0f)
	{
		current_ki = 150.0f;
	}
	if (rated_current <= 0.0f)
	{
		rated_current = DEFAULT_MOTOR_RATED_CURRENT_RMS;
	}
	align_current = rated_current * ID_SQUARE_TUNING_ALIGN_CURRENT_RATIO;
	if (align_current < ID_SQUARE_TUNING_ALIGN_CURRENT_MIN_A)
	{
		align_current = ID_SQUARE_TUNING_ALIGN_CURRENT_MIN_A;
	}

	gEncoderAlignmentRequested = 1u;
	gEncoderAlignmentContinueToRun = (continue_to_run != 0u) ? 1u : 0u;
	gEncoderAlignmentResumeRunMode = resume_run_mode;
	gEncoderAlignmentStatus = ENCODER_ALIGNMENT_STATUS_REQUESTED;
	gRunMode = RUN_MODE_ALIGNMENT_ONLY;
	gTargetSpeedRpm = 0.0f;
	gVfFrequencyHz = 0.0f;
	gVfVoltageV = 0.0f;
	gIdRefA = 0.0f;
	gIqRefA = 0.0f;
	IdSquareTuning.Enable = 1u;
	IdSquareTuning.Mode = ID_SQUARE_TUNING_MODE_ALIGNMENT_HOLD;
	IdSquareTuning.AlignmentDone = 0u;
	IdSquareTuning.ElectricalAngleTestMode = ID_SQUARE_ANGLE_TEST_NONE;
	IdSquareTuning.CurrentPolarityInvertTest = FORCE_DRIVER_CURRENT_POLARITY_INVERT;
	IdSquareTuning.CurrentUvSwapTest = FORCE_DRIVER_CURRENT_UV_SWAP;
	IdSquareTuning.AlignmentCounter = 0u;
	IdSquareTuning.fAmplitudeCmd = align_current;
	IdSquareTuning.fFrequencyCmd = 0.0f;
	IdSquareTuning.fCurrentKpCmd = current_kp;
	IdSquareTuning.fCurrentKiCmd = current_ki;
	IdSquareTuning.fAmplitudeApplied = 0.0f;
	IdSquareTuning.fFrequencyApplied = 0.0f;
	IdSquareTuning.fPhase = 0.0f;
	IdSquareTuning.fVoltageLimitApplied = 0.0f;
	IdSquareTuning.fAlignmentCurrentApplied = 0.0f;
	IdSquareTuning.OffsetCaptured = 0;
	ResetEncoderAlignmentAveraging();
}

static void FinalizeEncoderAlignment(uint8_t alignment_successful)
{
	uint8_t continue_to_run = gEncoderAlignmentContinueToRun;
	uint8_t resume_run_mode = gEncoderAlignmentResumeRunMode;

	gEncoderAlignmentRequested = 0u;
	gEncoderAlignmentContinueToRun = 0u;
	gEncoderAlignmentResumeRunMode = RUN_MODE_FOC;

	if (alignment_successful != 0u)
	{
		gEncoderAlignmentLastCapturedOffset = Parameter.Offset_Enc;
		gEncoderAlignmentStatus = ENCODER_ALIGNMENT_STATUS_DONE;
		if (gEncoderAlignmentPolicy == ENCODER_ALIGNMENT_POLICY_MANUAL_SAVE)
		{
			gEncoderAlignmentNeedsFlashSave = 1u;
		}
		else
		{
			gEncoderAlignmentNeedsFlashSave = 0u;
		}
	}
	else
	{
		gEncoderAlignmentStatus = ENCODER_ALIGNMENT_STATUS_FAULT;
	}

	IdSquareTuning.Enable = 0u;
	IdSquareTuning.AlignmentDone = 0u;
	IdSquareTuning.AlignmentCounter = 0u;
	IdSquareTuning.fPhase = 0.0f;
	IdSquareTuning.fVoltageLimitApplied = 0.0f;
	IdSquareTuning.fAlignmentCurrentApplied = 0.0f;
	ResetEncoderAlignmentAveraging();
	ResetControlLoops();

	if ((alignment_successful != 0u) && (continue_to_run != 0u))
	{
		gRunMode = resume_run_mode;
		STM_NextState(&StateMachine, START);
	}
	else
	{
		gRunMode = RUN_MODE_FOC;
		STM_NextState(&StateMachine, STOP);
	}
}

static uint32_t FlashAddressToSector(uint32_t address)
{
	if (address < 0x08004000u)
	{
		return FLASH_SECTOR_0;
	}
	if (address < 0x08008000u)
	{
		return FLASH_SECTOR_1;
	}
	if (address < 0x0800C000u)
	{
		return FLASH_SECTOR_2;
	}
	if (address < 0x08010000u)
	{
		return FLASH_SECTOR_3;
	}
	if (address < 0x08020000u)
	{
		return FLASH_SECTOR_4;
	}
	if (address < 0x08040000u)
	{
		return FLASH_SECTOR_5;
	}
	if (address < 0x08060000u)
	{
		return FLASH_SECTOR_6;
	}
	if (address < 0x08080000u)
	{
		return FLASH_SECTOR_7;
	}
	return FLASH_SECTOR_11;
}

static uint8_t FlashRegionHasData(uint32_t address, uint32_t word_count)
{
	uint32_t index;

	for (index = 0u; index < word_count; index++)
	{
		if (*((__IO uint32_t *)(address + (index * 4u))) != 0xFFFFFFFFu)
		{
			return 1u;
		}
	}

	return 0u;
}

static void LoadFloatArrayFromFlash(uint32_t address, float *destination, uint32_t count)
{
	uint32_t index;
	union
	{
		uint32_t u32;
		float f32;
	} value;

	for (index = 0u; index < count; index++)
	{
		value.u32 = *((__IO uint32_t *)(address + (index * 4u)));
		destination[index] = value.f32;
	}
}

static uint8_t FlashWriteFloatArray(uint32_t address, const float *source, uint32_t count)
{
	FLASH_EraseInitTypeDef erase_init;
	uint32_t sector_error = 0u;
	uint32_t index;
	union
	{
		uint32_t u32;
		float f32;
	} value;

	memset(&erase_init, 0, sizeof(erase_init));
	erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
	erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	erase_init.Sector = FlashAddressToSector(address);
	erase_init.NbSectors = 1u;

	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
		FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
	if (HAL_FLASHEx_Erase(&erase_init, &sector_error) != HAL_OK)
	{
		return 0u;
	}

	for (index = 0u; index < count; index++)
	{
		value.f32 = source[index];
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + (index * 4u), value.u32) != HAL_OK)
		{
			return 0u;
		}
	}

	return 1u;
}

static uint8_t LoadParametersFromFlashIfAvailable(void)
{
	uint8_t loaded = 0u;

	if (FlashRegionHasData(PAGE_ADDRESS_DRIVER_PARAMETER, DRIVER_PARAMETER_COUNT) != 0u)
	{
		LoadFloatArrayFromFlash(PAGE_ADDRESS_DRIVER_PARAMETER, DriverParameter, DRIVER_PARAMETER_COUNT);
		loaded = 1u;
	}
	if (FlashRegionHasData(PAGE_ADDRESS_MOTOR_PARAMETER, MOTOR_PARAMETER_COUNT) != 0u)
	{
		LoadFloatArrayFromFlash(PAGE_ADDRESS_MOTOR_PARAMETER, MotorParameter, MOTOR_PARAMETER_COUNT);
		loaded = 1u;
	}

	return loaded;
}

uint8_t SaveParametersToFlash(void)
{
	uint8_t success;

	if ((StateMachine.bState == RUN) || (StateMachine.bState == ENCODER_ALIGN) ||
		(StateMachine.bState == OFFSET_CALIB) || (StateMachine.bState == START))
	{
		return 0u;
	}

	HAL_FLASH_Unlock();
	success = FlashWriteFloatArray(PAGE_ADDRESS_DRIVER_PARAMETER, DriverParameter, DRIVER_PARAMETER_COUNT);
	if (success != 0u)
	{
		success = FlashWriteFloatArray(PAGE_ADDRESS_MOTOR_PARAMETER, MotorParameter, MOTOR_PARAMETER_COUNT);
	}
	HAL_FLASH_Lock();

	if (success != 0u)
	{
		gEncoderAlignmentNeedsFlashSave = 0u;
		gEncoderAlignmentLastCapturedOffset = Parameter.Offset_Enc;
	}

	return success;
}

static float GetEffectiveCurrentLoopFrequency(void)
{
	if (gEffectiveCurrentLoopFrequencyHz > 1.0f)
	{
		return gEffectiveCurrentLoopFrequencyHz;
	}
	return USER_ISR_FREQUENCY_16KHZ;
}

static float GetEffectiveSpeedLoopFrequency(void)
{
	if (gEffectiveSpeedLoopFrequencyHz > 1.0f)
	{
		return gEffectiveSpeedLoopFrequencyHz;
	}
	return GetEffectiveCurrentLoopFrequency() * 0.5f;
}

void ApplyControlTimingMode(uint8_t mode)
{
	float current_loop_hz;

	(void)mode;
	mode = CONTROL_TIMING_MODE_16KHZ;
	current_loop_hz = USER_ISR_FREQUENCY_16KHZ;

	gControlTimingMode = mode;
	gEffectiveCurrentLoopFrequencyHz = current_loop_hz;
	gEffectiveSpeedLoopFrequencyHz = current_loop_hz * 0.5f;
	Parameter.Ui16ControlFrequency = (uint16_t)gEffectiveCurrentLoopFrequencyHz;

	gSpeedPi.fDtSec = 1.0f / GetEffectiveSpeedLoopFrequency();
	gIdPi.fDtSec = 1.0f / GetEffectiveCurrentLoopFrequency();
	gIqPi.fDtSec = 1.0f / GetEffectiveCurrentLoopFrequency();

	IdSquareTuning.fPhase = 0.0f;
	IdSquareTuning.AlignmentCounter = 0u;
	ResetDebugAveraging();
	ResetControl_V_over_F();
}

static void InitIsrDebugCounter(void)
{
	SystemCoreClockUpdate();
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0u;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	sLastIsrCycleCount = DWT->CYCCNT;
	sIsrDebugCounterReady = 1u;
	gDebugIsrDeltaCycles = 0u;
	gDebugIsrPeriodUs = 0.0f;
	gDebugIsrFrequencyHz = 0.0f;
}

static void UpdateIsrDebugPeriod(void)
{
	uint32_t current_cycle_count;
	uint32_t delta_cycles;

	if (sIsrDebugCounterReady == 0u)
	{
		return;
	}

	current_cycle_count = DWT->CYCCNT;
	delta_cycles = current_cycle_count - sLastIsrCycleCount;
	sLastIsrCycleCount = current_cycle_count;
	gDebugIsrDeltaCycles = delta_cycles;

	if ((delta_cycles > 0u) && (SystemCoreClock > 0u))
	{
		gDebugIsrFrequencyHz = (float)SystemCoreClock / (float)delta_cycles;
		gDebugIsrPeriodUs = ((float)delta_cycles * 1000000.0f) / (float)SystemCoreClock;
	}
}

static void UpdateIsrMeasureOnlyFrequency(void)
{
	uint32_t now_tick_ms;
	uint32_t delta_tick_ms;
	uint32_t delta_count;

	if (gIsrMeasureOnlyMode != sIsrMeasurePrevMode)
	{
		sIsrMeasurePrevMode = gIsrMeasureOnlyMode;
		sIsrMeasurePrevCount = gIsrMeasureEdgeCount;
		sIsrMeasurePrevTickMs = HAL_GetTick();
		gIsrMeasureEdgeFrequencyHz = 0.0f;
	}

	if (gIsrMeasureOnlyMode == 0u)
	{
		return;
	}

	now_tick_ms = HAL_GetTick();
	delta_tick_ms = now_tick_ms - sIsrMeasurePrevTickMs;
	if (delta_tick_ms < 200u)
	{
		return;
	}

	delta_count = gIsrMeasureEdgeCount - sIsrMeasurePrevCount;
	gIsrMeasureEdgeFrequencyHz = ((float)delta_count * 1000.0f) / (float)delta_tick_ms;
	sIsrMeasurePrevCount = gIsrMeasureEdgeCount;
	sIsrMeasurePrevTickMs = now_tick_ms;
}

static void SetPwmEnabled(uint8_t enable)
{
	if ((enable != 0u) && (gPwmEnabled == 0u))
	{
		SwitchOnPWM();
		gPwmEnabled = 1u;
	}
	else if ((enable == 0u) && (gPwmEnabled != 0u))
	{
		SwitchOffPWM();
		gPwmEnabled = 0u;
	}
}

static void ResetControlLoops(void)
{
	gIdPi.m_rst(&gIdPi);
	gIqPi.m_rst(&gIqPi);
	gSpeedPi.m_rst(&gSpeedPi);
	gPositionPi.m_rst(&gPositionPi);
	gIdRefA = 0.0f;
	gIqRefA = 0.0f;
	gCommandedSpeedRpm = 0.0f;
	gTracePosError = 0.0f;
	gSpeedLoopDivider = 0u;
	sPositionSetpointPrevCounts = gTargetPositionCounts;
	sPositionSetpointVelocityCountsPerSec = 0.0f;
	sPositionDeadbandHoldActive = 0u;
	gDebugOpenLoopElectricalHzCmd = 0.0f;
	gDebugOpenLoopSyncRpmCmd = 0.0f;
	gDebugExpectedDeltaPosSync = 0.0f;
	Parameter.fVdq[0] = 0.0f;
	Parameter.fVdq[1] = 0.0f;
	Parameter.fVabc[0] = 0.0f;
	Parameter.fVabc[1] = 0.0f;
	Parameter.fVabc[2] = 0.0f;
	ResetDebugAveraging();
	ResetControl_V_over_F();
	GeneratePWM(0.5f, 0.5f, 0.5f);
}

static uint8_t GetActiveFocControlMode(void)
{
	if ((uint8_t)DriverParameter[CONTROL_MODE] == POSITION_CONTROL_MODE)
	{
		return POSITION_CONTROL_MODE;
	}
	return SPEED_CONTROL_MODE;
}

static float GetConfiguredPositionGain(void)
{
	float position_gain = DriverParameter[POSITION_P_GAIN];
	if (position_gain < 0.0f)
	{
		position_gain = 0.0f;
	}
	return position_gain;
}

static float GetConfiguredPositionIntegralGain(void)
{
	float position_integral_gain = DriverParameter[POSITION_I_GAIN];
	if (position_integral_gain < 0.0f)
	{
		position_integral_gain = 0.0f;
	}
	return position_integral_gain;
}

static float GetConfiguredPositionVffGain(void)
{
	float position_vff_gain = DriverParameter[POSITION_FF_GAIN];
	if (position_vff_gain < 0.0f)
	{
		position_vff_gain = 0.0f;
	}
	return position_vff_gain;
}

static float GetConfiguredPositionVffFilterHz(void)
{
	float position_vff_filter_hz = DriverParameter[POSITION_FF_FILTER];
	if (position_vff_filter_hz <= 0.0f)
	{
		position_vff_filter_hz = POSITION_VFF_FILTER_DEFAULT_HZ;
	}
	return position_vff_filter_hz;
}

static uint8_t GetConfiguredPositionTrackingMode(void)
{
	uint8_t tracking_mode = (uint8_t)DriverParameter[POSITION_TRACKING_MODE];
	if (tracking_mode > POSITION_TRACKING_MODE_MULTI_TURN)
	{
		tracking_mode = POSITION_TRACKING_MODE_SINGLE_TURN;
	}
	return tracking_mode;
}

static float GetConfiguredSpeedLimitRpm(void)
{
	float speed_limit_rpm = DriverParameter[MAXIMUM_SPEED];
	if (speed_limit_rpm <= 0.0f)
	{
		speed_limit_rpm = MotorParameter[MOTOR_MAXIMUM_SPEED];
	}
	if (speed_limit_rpm <= 0.0f)
	{
		speed_limit_rpm = DEFAULT_MOTOR_RATED_SPEED_RPM;
	}
	return speed_limit_rpm;
}

static float GetConfiguredSpeedIqLimitA(void)
{
	float rated_limit = MotorParameter[MOTOR_RATED_CURRENT_RMS];
	float oc_limit = Current_Sensor.OverCurrentThreshold * FOC_SPEED_LOOP_MAX_OC_RATIO;
	float iq_limit = rated_limit;

	if ((oc_limit > 0.0f) && ((iq_limit <= 0.0f) || (oc_limit < iq_limit)))
	{
		iq_limit = oc_limit;
	}
	if (iq_limit <= 0.0f)
	{
		iq_limit = FOC_SPEED_LOOP_MIN_IQ_LIMIT_A;
	}
	if (iq_limit < FOC_SPEED_LOOP_MIN_IQ_LIMIT_A)
	{
		iq_limit = FOC_SPEED_LOOP_MIN_IQ_LIMIT_A;
	}
	return iq_limit;
}

static float WrapPositionErrorCounts(float error_counts, float encoder_resolution)
{
	float half_encoder_resolution;

	if (encoder_resolution <= 1.0f)
	{
		return error_counts;
	}

	error_counts = fmodf(error_counts, encoder_resolution);
	half_encoder_resolution = 0.5f * encoder_resolution;
	if (error_counts > half_encoder_resolution)
	{
		error_counts -= encoder_resolution;
	}
	if (error_counts < -half_encoder_resolution)
	{
		error_counts += encoder_resolution;
	}
	return error_counts;
}

static float GetPositionControlErrorCounts(float target_position_counts, float actual_position_counts, float encoder_resolution)
{
	if (GetConfiguredPositionTrackingMode() == POSITION_TRACKING_MODE_MULTI_TURN)
	{
		return target_position_counts - actual_position_counts;
	}

	return WrapPositionErrorCounts(
		target_position_counts - actual_position_counts,
		encoder_resolution);
}

static float GetPositionLoopErrorDeadbandCounts(float encoder_resolution)
{
	if (encoder_resolution <= 0.0f)
	{
		return 0.0f;
	}

	return (POSITION_LOOP_ERROR_DEADBAND_DEG / 360.0f) * encoder_resolution;
}

static float GetPositionLoopErrorReleaseDeadbandCounts(float encoder_resolution)
{
	if (encoder_resolution <= 0.0f)
	{
		return 0.0f;
	}

	return (POSITION_LOOP_ERROR_RELEASE_DEADBAND_DEG / 360.0f) * encoder_resolution;
}

static float UpdatePositionSetpointVelocityRpm(float target_position_counts, float encoder_resolution, float dt_sec)
{
	float target_delta_counts;
	float target_velocity_counts_per_sec;
	float filter_hz;
	float filter_tau_sec;
	float filter_alpha;

	if ((encoder_resolution <= 1.0f) || (dt_sec <= 0.0f))
	{
		sPositionSetpointPrevCounts = target_position_counts;
		sPositionSetpointVelocityCountsPerSec = 0.0f;
		return 0.0f;
	}

	if (GetConfiguredPositionTrackingMode() == POSITION_TRACKING_MODE_MULTI_TURN)
	{
		target_delta_counts = target_position_counts - sPositionSetpointPrevCounts;
	}
	else
	{
		target_delta_counts = WrapPositionErrorCounts(
			target_position_counts - sPositionSetpointPrevCounts,
			encoder_resolution);
	}
	sPositionSetpointPrevCounts = target_position_counts;
	target_velocity_counts_per_sec = target_delta_counts / dt_sec;
	filter_hz = GetConfiguredPositionVffFilterHz();
	if (filter_hz > 0.0f)
	{
		filter_tau_sec = 1.0f / (2.0f * PI * filter_hz);
		filter_alpha = dt_sec / (filter_tau_sec + dt_sec);
		filter_alpha = ClampFloat(filter_alpha, 0.0f, 1.0f);
		sPositionSetpointVelocityCountsPerSec += filter_alpha *
			(target_velocity_counts_per_sec - sPositionSetpointVelocityCountsPerSec);
	}
	else
	{
		sPositionSetpointVelocityCountsPerSec = target_velocity_counts_per_sec;
	}

	return (sPositionSetpointVelocityCountsPerSec * 60.0f) / encoder_resolution;
}

static float ApplySpeedRampLimit(float current_command, float target_command, float max_speed_rpm, float dt_sec)
{
	float accel_ms = DriverParameter[ACCELERATION_TIME];
	float decel_ms = DriverParameter[DECELERATION_TIME];
	float accel_step;
	float decel_step;
	float step_limit;
	float delta;

	if ((dt_sec <= 0.0f) || (max_speed_rpm <= 0.0f))
	{
		return target_command;
	}

	if (accel_ms <= 0.0f)
	{
		accel_ms = 200.0f;
	}
	if (decel_ms <= 0.0f)
	{
		decel_ms = accel_ms;
	}

	accel_step = max_speed_rpm * dt_sec / (accel_ms * 0.001f);
	decel_step = max_speed_rpm * dt_sec / (decel_ms * 0.001f);
	if (accel_step <= 0.0f)
	{
		accel_step = max_speed_rpm;
	}
	if (decel_step <= 0.0f)
	{
		decel_step = max_speed_rpm;
	}

	if ((current_command * target_command) < 0.0f)
	{
		step_limit = decel_step;
	}
	else if (fabsf(target_command) > fabsf(current_command))
	{
		step_limit = accel_step;
	}
	else
	{
		step_limit = decel_step;
	}

	delta = target_command - current_command;
	if (delta > step_limit)
	{
		delta = step_limit;
	}
	else if (delta < -step_limit)
	{
		delta = -step_limit;
	}

	return current_command + delta;
}

static void LimitDqVoltageVector(float *vd, float *vq, float limit)
{
	float magnitude_sq;
	float limit_sq;
	float scale;

	if ((vd == 0) || (vq == 0) || (limit <= 0.0f))
	{
		return;
	}

	magnitude_sq = (*vd * *vd) + (*vq * *vq);
	limit_sq = limit * limit;
	if (magnitude_sq <= limit_sq)
	{
		return;
	}

	scale = limit / sqrtf(magnitude_sq);
	*vd *= scale;
	*vq *= scale;
}

static float CalcIdSquareTuningVoltageLimit(void)
{
	float voltage_ratio = ID_SQUARE_TUNING_ALIGN_VOLTAGE_RATIO;
	float voltage_limit;
	float motor_voltage_limit;

	if ((IdSquareTuning.Enable != 0u) &&
		(IdSquareTuning.Mode == ID_SQUARE_TUNING_MODE_SQUARE_WAVE) &&
		(IdSquareTuning.AlignmentDone != 0u))
	{
		voltage_ratio = ID_SQUARE_TUNING_SQUARE_VOLTAGE_RATIO;
	}

	voltage_limit = 0.5f * Parameter.fVdc * voltage_ratio;
	motor_voltage_limit = MotorParameter[MOTOR_MAXIMUM_VOLTAGE] * voltage_ratio;

	if ((motor_voltage_limit > 0.0f) && ((voltage_limit <= 0.0f) || (motor_voltage_limit < voltage_limit)))
	{
		voltage_limit = motor_voltage_limit;
	}
	if (voltage_limit < ID_SQUARE_TUNING_MIN_VOLTAGE_LIMIT_V)
	{
		voltage_limit = ID_SQUARE_TUNING_MIN_VOLTAGE_LIMIT_V;
	}

	return voltage_limit;
}

static float CalcIdSquareTuningReference(void)
{
	float rated_limit = MotorParameter[MOTOR_RATED_CURRENT_RMS] * ID_SQUARE_TUNING_MAX_RATED_CURRENT_RATIO;
	float oc_limit = Current_Sensor.OverCurrentThreshold * ID_SQUARE_TUNING_MAX_OC_RATIO;
	float amplitude_limit = rated_limit;

	if ((oc_limit > 0.0f) && ((amplitude_limit <= 0.0f) || (oc_limit < amplitude_limit)))
	{
		amplitude_limit = oc_limit;
	}
	if (amplitude_limit < 0.1f)
	{
		amplitude_limit = 0.1f;
	}

	IdSquareTuning.fAmplitudeApplied = ClampFloat(
		fabsf(IdSquareTuning.fAmplitudeCmd),
		0.0f,
		amplitude_limit);
	IdSquareTuning.fFrequencyApplied = ClampFloat(
		IdSquareTuning.fFrequencyCmd,
		0.5f,
		ID_SQUARE_TUNING_MAX_FREQUENCY_HZ);

	if ((IdSquareTuning.Enable == 0u) || (IdSquareTuning.fAmplitudeApplied < 0.0001f))
	{
		IdSquareTuning.fPhase = 0.0f;
		return 0.0f;
	}

	IdSquareTuning.fPhase += IdSquareTuning.fFrequencyApplied / GetEffectiveCurrentLoopFrequency();
	if (IdSquareTuning.fPhase >= 1.0f)
	{
		IdSquareTuning.fPhase -= 1.0f;
	}

//	return (IdSquareTuning.fPhase < 0.5f) ? IdSquareTuning.fAmplitudeApplied : -IdSquareTuning.fAmplitudeApplied;
	return (IdSquareTuning.fPhase < 0.5f) ? IdSquareTuning.fAmplitudeApplied : 0.0f;
}

//static float CalcIdSquareTuningReference(void)
//{
//    // 1. Gi? nguyên ph?n tính toán gi?i h?n an toàn d? b?o v? Motor/Driver
//    float rated_limit = MotorParameter[MOTOR_RATED_CURRENT_RMS] * ID_SQUARE_TUNING_MAX_RATED_CURRENT_RATIO;
//    float oc_limit = Current_Sensor.OverCurrentThreshold * ID_SQUARE_TUNING_MAX_OC_RATIO;
//    float amplitude_limit = rated_limit;

//    if ((oc_limit > 0.0f) && ((amplitude_limit <= 0.0f) || (oc_limit < amplitude_limit)))
//    {
//        amplitude_limit = oc_limit;
//    }
//    if (amplitude_limit < 0.1f)
//    {
//        amplitude_limit = 0.1f;
//    }

//    // Gán biên d? t? l?nh ngu?i dùng và Clamp l?i cho an toàn
//    IdSquareTuning.fAmplitudeApplied = ClampFloat(
//        fabsf(IdSquareTuning.fAmplitudeCmd),
//        0.0f,
//        amplitude_limit);

//    // 2. Ki?m tra di?u ki?n Enable - N?u không b?t thì tr? v? 0
//    if ((IdSquareTuning.Enable == 0u) || (IdSquareTuning.fAmplitudeApplied < 0.0001f))
//    {
//        IdSquareTuning.fPhase = 0.0f;
//        return 0.0f;
//    }

//    // 3. THAY Ð?I CHÍNH ? ÐÂY: 
//    // Không c?ng d?n fPhase n?a, ép nó b?ng 0 d? không t?o ra chu k? xung.
//    IdSquareTuning.fPhase = 0.0f;
//    IdSquareTuning.fFrequencyApplied = 0.0f;

//    // 4. TR? V? TH?NG BIÊN Ð?:
//    // Thay vì dùng (fPhase < 0.5f) d? b?t/t?t dòng di?n, ta tr? v? h?ng s? luôn.
//    return IdSquareTuning.fAmplitudeApplied;
//}

static float CalcIdSquareTuningAlignmentCurrent(void)
{
	float rated_limit = MotorParameter[MOTOR_RATED_CURRENT_RMS] * ID_SQUARE_TUNING_MAX_RATED_CURRENT_RATIO;
	float oc_limit = Current_Sensor.OverCurrentThreshold * ID_SQUARE_TUNING_MAX_OC_RATIO;
	float amplitude_limit = rated_limit;
	float align_current;

	if ((oc_limit > 0.0f) && ((amplitude_limit <= 0.0f) || (oc_limit < amplitude_limit)))
	{
		amplitude_limit = oc_limit;
	}
	if (amplitude_limit < 0.1f)
	{
		amplitude_limit = 0.1f;
	}

	IdSquareTuning.fAmplitudeApplied = ClampFloat(
		fabsf(IdSquareTuning.fAmplitudeCmd),
		0.0f,
		amplitude_limit);
	align_current = IdSquareTuning.fAmplitudeApplied;
	if (align_current < ID_SQUARE_TUNING_ALIGN_CURRENT_MIN_A)
	{
		align_current = ID_SQUARE_TUNING_ALIGN_CURRENT_MIN_A;
	}
	if (align_current > amplitude_limit)
	{
		align_current = amplitude_limit;
	}
	IdSquareTuning.fAlignmentCurrentApplied = align_current;
	return align_current;
}

static void ResetEncoderAlignmentAveraging(void)
{
	sEncoderAlignSinAccum = 0.0f;
	sEncoderAlignCosAccum = 0.0f;
	sEncoderAlignSampleCount = 0u;
}

static float GetAlignedSingleTurnPositionCounts(float encoder_resolution)
{
	float single_turn_position;

	if (encoder_resolution <= 1.0f)
	{
		return 0.0f;
	}

	if (MotorParameter[MOTOR_CURRENT_CTRL_DIRECTION] == 0.0f)
	{
		single_turn_position = (float)Parameter.EncSingleTurn;
	}
	else
	{
		single_turn_position = encoder_resolution - (float)Parameter.EncSingleTurn;
	}

	single_turn_position = fmodf(single_turn_position, encoder_resolution);
	if (single_turn_position < 0.0f)
	{
		single_turn_position += encoder_resolution;
	}

	return single_turn_position;
}

static int32_t CalcAlignedEncoderOffsetFromCounts(float single_turn_position, float encoder_resolution)
{
	float pole_pairs;
	float electrical_offset_counts;
	int32_t offset_counts;

	if (encoder_resolution <= 1.0f)
	{
		return 0;
	}

	pole_pairs = (MotorParameter[MOTOR_NUMBER_POLE_PAIRS] > 0.0f) ?
		MotorParameter[MOTOR_NUMBER_POLE_PAIRS] : (float)INITIAL_MOTOR_POLE_PAIRS;
	if (pole_pairs <= 0.0f)
	{
		pole_pairs = 1.0f;
	}

	if (single_turn_position > (encoder_resolution * 0.5f))
	{
		single_turn_position -= encoder_resolution;
	}

	offset_counts = (int32_t)lroundf(-single_turn_position);
	electrical_offset_counts = (encoder_resolution * DEFAULT_ELECTRICAL_ALIGNMENT_OFFSET_DEG) /
		(360.0f * pole_pairs);
	offset_counts += (int32_t)lroundf(electrical_offset_counts);
	if (offset_counts > (int32_t)(encoder_resolution * 0.5f))
	{
		offset_counts -= (int32_t)encoder_resolution;
	}
	else if (offset_counts < -(int32_t)(encoder_resolution * 0.5f))
	{
		offset_counts += (int32_t)encoder_resolution;
	}

	return offset_counts;
}

static void AccumulateEncoderAlignmentSample(float encoder_resolution)
{
	float single_turn_position;
	float sample_angle;

	if (encoder_resolution <= 1.0f)
	{
		return;
	}

	single_turn_position = GetAlignedSingleTurnPositionCounts(encoder_resolution);
	sample_angle = (single_turn_position / encoder_resolution) * (2.0f * PI);
	sEncoderAlignSinAccum += sinf(sample_angle);
	sEncoderAlignCosAccum += cosf(sample_angle);
	sEncoderAlignSampleCount++;
}

static int32_t CalcAveragedAlignedEncoderOffset(float encoder_resolution)
{
	float average_angle;
	float average_single_turn_position;

	if ((encoder_resolution <= 1.0f) || (sEncoderAlignSampleCount == 0u))
	{
		return CalcAlignedEncoderOffsetFromCounts(
			GetAlignedSingleTurnPositionCounts(encoder_resolution),
			encoder_resolution);
	}

	average_angle = atan2f(sEncoderAlignSinAccum, sEncoderAlignCosAccum);
	if (average_angle < 0.0f)
	{
		average_angle += (2.0f * PI);
	}
	average_single_turn_position = (average_angle * encoder_resolution) / (2.0f * PI);
	return CalcAlignedEncoderOffsetFromCounts(average_single_turn_position, encoder_resolution);
}

static int32_t CalcAlignedEncoderOffset(float encoder_resolution)
{
	return CalcAlignedEncoderOffsetFromCounts(
		GetAlignedSingleTurnPositionCounts(encoder_resolution),
		encoder_resolution);
}

uint8_t StartFocRotatingThetaTest(void)
{
	if ((StateMachine.bState != IDLE) && (StateMachine.bState != STOP))
	{
		return 0u;
	}
	if (FaultCode != NO_ERROR)
	{
		return 0u;
	}
	if (Current_Sensor.CalibFinish <= 0)
	{
		return 0u;
	}
	if (gEncoderAlignmentStatus != ENCODER_ALIGNMENT_STATUS_DONE)
	{
		return 0u;
	}

	gRunMode = RUN_MODE_FOC;
	gVfFrequencyHz = 0.0f;
	gVfVoltageV = 0.0f;
	gTargetSpeedRpm = 0.0f;
	gTargetPositionCounts = Parameter.fPosition;
	gCommandedSpeedRpm = 0.0f;
	gTracePosError = 0.0f;
	gIdRefA = 0.0f;
	gIqRefA = 0.0f;
	gFocElectricalAngleTestMode = ID_SQUARE_ANGLE_TEST_NONE;
	gFocCurrentUvSwapTest = 0u;
	gFocCurrentPolarityInvertTest = 0u;
	gFocRotatingThetaVoltageTestRunning = 0u;
	gFocCurrentFeedbackMapTestRunning = 0u;
	gFocRotatingThetaTestRunning = 1u;
	gFocRotatingThetaDebugThetaDeg = 0.0f;
	gFocRotatingThetaDebugDeltaPos = 0;
	sFocRotatingThetaCommand = 0.0f;
	sFocRotatingThetaStartPosition = Parameter.fPosition;
	sFocRotatingThetaLogCounter = 0u;
	USB_QueueFocDebugText(
		"[RTH] start f=%.2f iq=%.3f frame=%.2f off=%ld",
		ROTATING_THETA_TEST_ELECTRICAL_FREQ_HZ,
		ROTATING_THETA_TEST_IQ_A,
		ROTATING_THETA_TEST_FRAME_DEG,
		(long)Parameter.Offset_Enc);
	FOC_DEBUG_PRINTF(
		"[RTH] start f=%.2fHz iq=%.3fA frame=%.2fdeg offset=%ld\r\n",
		ROTATING_THETA_TEST_ELECTRICAL_FREQ_HZ,
		ROTATING_THETA_TEST_IQ_A,
		ROTATING_THETA_TEST_FRAME_DEG,
		(long)Parameter.Offset_Enc);
	ResetControlLoops();
	STM_NextState(&StateMachine, START);
	return 1u;
}

uint8_t StartFocRotatingThetaVoltageTest(void)
{
	if ((StateMachine.bState != IDLE) && (StateMachine.bState != STOP))
	{
		return 0u;
	}
	if (FaultCode != NO_ERROR)
	{
		return 0u;
	}
	if (Current_Sensor.CalibFinish <= 0)
	{
		return 0u;
	}
	if (gEncoderAlignmentStatus != ENCODER_ALIGNMENT_STATUS_DONE)
	{
		return 0u;
	}

	gRunMode = RUN_MODE_FOC;
	gVfFrequencyHz = 0.0f;
	gVfVoltageV = 0.0f;
	gTargetSpeedRpm = 0.0f;
	gTargetPositionCounts = Parameter.fPosition;
	gCommandedSpeedRpm = 0.0f;
	gTracePosError = 0.0f;
	gIdRefA = 0.0f;
	gIqRefA = 0.0f;
	gFocElectricalAngleTestMode = ID_SQUARE_ANGLE_TEST_NONE;
	gFocCurrentUvSwapTest = 0u;
	gFocCurrentPolarityInvertTest = 0u;
	gFocRotatingThetaTestRunning = 0u;
	gFocCurrentFeedbackMapTestRunning = 0u;
	gFocRotatingThetaVoltageTestRunning = 1u;
	gFocRotatingThetaVoltageDebugThetaDeg = 0.0f;
	gFocRotatingThetaVoltageDebugDeltaPos = 0;
	sFocRotatingThetaVoltageCommand = 0.0f;
	sFocRotatingThetaVoltageStartPosition = Parameter.fPosition;
	sFocRotatingThetaVoltageLogCounter = 0u;
	USB_QueueFocDebugText(
		"[RTV] start f=%.2f vq=%.2f frame=%.2f off=%ld",
		ROTATING_THETA_VOLTAGE_TEST_ELECTRICAL_FREQ_HZ,
		ROTATING_THETA_VOLTAGE_TEST_VQ_V,
		ROTATING_THETA_VOLTAGE_TEST_FRAME_DEG,
		(long)Parameter.Offset_Enc);
	FOC_DEBUG_PRINTF(
		"[RTV] start f=%.2fHz vq=%.2fV frame=%.2fdeg offset=%ld\r\n",
		ROTATING_THETA_VOLTAGE_TEST_ELECTRICAL_FREQ_HZ,
		ROTATING_THETA_VOLTAGE_TEST_VQ_V,
		ROTATING_THETA_VOLTAGE_TEST_FRAME_DEG,
		(long)Parameter.Offset_Enc);
	ResetControlLoops();
	STM_NextState(&StateMachine, START);
	return 1u;
}

uint8_t StartFocCurrentFeedbackMapTest(void)
{
	if ((StateMachine.bState != IDLE) && (StateMachine.bState != STOP))
	{
		return 0u;
	}
	if (FaultCode != NO_ERROR)
	{
		return 0u;
	}
	if (Current_Sensor.CalibFinish <= 0)
	{
		return 0u;
	}
	if (gEncoderAlignmentStatus != ENCODER_ALIGNMENT_STATUS_DONE)
	{
		return 0u;
	}

	gRunMode = RUN_MODE_FOC;
	gVfFrequencyHz = 0.0f;
	gVfVoltageV = 0.0f;
	gTargetSpeedRpm = 0.0f;
	gTargetPositionCounts = Parameter.fPosition;
	gCommandedSpeedRpm = 0.0f;
	gTracePosError = 0.0f;
	gIdRefA = 0.0f;
	gIqRefA = 0.0f;
	gFocElectricalAngleTestMode = ID_SQUARE_ANGLE_TEST_NONE;
	gFocCurrentUvSwapTest = sFocCurrentFeedbackMapSwapCases[0];
	gFocCurrentPolarityInvertTest = sFocCurrentFeedbackMapInvertCases[0];
	gFocRotatingThetaTestRunning = 0u;
	gFocRotatingThetaVoltageTestRunning = 0u;
	gFocCurrentFeedbackMapTestRunning = 1u;
	sFocCurrentFeedbackMapCaseIndex = 0u;
	sFocCurrentFeedbackMapCounter = 0u;
	sFocCurrentFeedbackMapCommand = 0.0f;
	sFocCurrentFeedbackMapStartPosition = Parameter.fPosition;
	sFocCurrentFeedbackMapIdAbsAccum = 0.0f;
	sFocCurrentFeedbackMapIqAccum = 0.0f;
	sFocCurrentFeedbackMapIqErrAbsAccum = 0.0f;
	sFocCurrentFeedbackMapSpeedAbsAccum = 0.0f;
	sFocCurrentFeedbackMapSampleCount = 0u;
	memset(sFocCurrentFeedbackMapIdAbsAvg, 0, sizeof(sFocCurrentFeedbackMapIdAbsAvg));
	memset(sFocCurrentFeedbackMapIqAvg, 0, sizeof(sFocCurrentFeedbackMapIqAvg));
	memset(sFocCurrentFeedbackMapIqErrAbsAvg, 0, sizeof(sFocCurrentFeedbackMapIqErrAbsAvg));
	memset(sFocCurrentFeedbackMapSpeedAbsAvg, 0, sizeof(sFocCurrentFeedbackMapSpeedAbsAvg));
	memset(sFocCurrentFeedbackMapDeltaPos, 0, sizeof(sFocCurrentFeedbackMapDeltaPos));
	USB_QueueFocDebugText(
		"[CFM] start f=%.2f iq=%.3f frame=%.2f off=%ld",
		ROTATING_THETA_TEST_ELECTRICAL_FREQ_HZ,
		ROTATING_THETA_TEST_IQ_A,
		ROTATING_THETA_TEST_FRAME_DEG,
		(long)Parameter.Offset_Enc);
	USB_QueueFocDebugText(
		"[CFM] case=%u name=%s swap=%u inv=%u begin",
		(unsigned int)sFocCurrentFeedbackMapCaseIndex,
		GetCurrentFeedbackMapCaseName(sFocCurrentFeedbackMapCaseIndex),
		(unsigned int)gFocCurrentUvSwapTest,
		(unsigned int)gFocCurrentPolarityInvertTest);
	FOC_DEBUG_PRINTF(
		"[CFM] start f=%.2fHz iq=%.3fA frame=%.2fdeg offset=%ld\r\n",
		ROTATING_THETA_TEST_ELECTRICAL_FREQ_HZ,
		ROTATING_THETA_TEST_IQ_A,
		ROTATING_THETA_TEST_FRAME_DEG,
		(long)Parameter.Offset_Enc);
	ResetControlLoops();
	STM_NextState(&StateMachine, START);
	return 1u;
}

void StopFocDiagnosticModes(void)
{
	gFocRotatingThetaTestRunning = 0u;
	gFocRotatingThetaVoltageTestRunning = 0u;
	gFocCurrentFeedbackMapTestRunning = 0u;
	gFocCurrentUvSwapTest = 0u;
	gFocCurrentPolarityInvertTest = 0u;
}
static void RunFocRotatingThetaTestLoop(void)
{
	float voltage_limit;
	float theta_step_rad;
	float control_theta;
	uint16_t log_samples;

	voltage_limit = Parameter.fVdc * ROTATING_THETA_TEST_VOLTAGE_LIMIT_RATIO;
	if (voltage_limit < ROTATING_THETA_TEST_MIN_VOLTAGE_LIMIT_V)
	{
		voltage_limit = ROTATING_THETA_TEST_MIN_VOLTAGE_LIMIT_V;
	}
	if (voltage_limit > ROTATING_THETA_TEST_MAX_VOLTAGE_LIMIT_V)
	{
		voltage_limit = ROTATING_THETA_TEST_MAX_VOLTAGE_LIMIT_V;
	}
	theta_step_rad =
		(2.0f * PI * ROTATING_THETA_TEST_ELECTRICAL_FREQ_HZ) /
		GetEffectiveCurrentLoopFrequency();
	log_samples = (uint16_t)ClampFloat(
		GetEffectiveCurrentLoopFrequency() * ROTATING_THETA_TEST_LOG_TIME_S,
		1.0f,
		60000.0f);

	LoadDiagnosticCurrentPiGains();
	gIdRefA = 0.0f;
	gIqRefA = ROTATING_THETA_TEST_IQ_A;
	gTargetSpeedRpm = 0.0f;
	gCommandedSpeedRpm = 0.0f;
	gTracePosError = 0.0f;
	sFocRotatingThetaCommand = WrapAngle(sFocRotatingThetaCommand + theta_step_rad);
	control_theta = WrapAngle(
		sFocRotatingThetaCommand + ((ROTATING_THETA_TEST_FRAME_DEG * PI) / 180.0f));
	gFocRotatingThetaDebugThetaDeg = control_theta * (180.0f / PI);
	UpdateMeasuredCurrentsForTheta(control_theta, Parameter.fIabc[0], Parameter.fIabc[1]);
	RunCurrentLoopForTheta(control_theta, 0.0f, ROTATING_THETA_TEST_IQ_A, voltage_limit, 0u);
	gFocRotatingThetaDebugDeltaPos =
		(int32_t)lroundf(Parameter.fPosition - sFocRotatingThetaStartPosition);

	if (++sFocRotatingThetaLogCounter >= log_samples)
	{
		sFocRotatingThetaLogCounter = 0u;
		USB_QueueFocDebugText(
			"[RTH] th=%.1f id=%.3f iq=%.3f pos=%ld sp=%.1f v=%.2f",
			gFocRotatingThetaDebugThetaDeg,
			Parameter.fIdq[0],
			Parameter.fIdq[1],
			(long)gFocRotatingThetaDebugDeltaPos,
			Parameter.fActSpeed,
			voltage_limit);
		FOC_DEBUG_PRINTF(
			"[RTH] theta=%.1f deg id=%.3f iq=%.3f pos=%ld speed=%.1f rpm vlim=%.2f\r\n",
			gFocRotatingThetaDebugThetaDeg,
			Parameter.fIdq[0],
			Parameter.fIdq[1],
			(long)gFocRotatingThetaDebugDeltaPos,
			Parameter.fActSpeed,
			voltage_limit);
	}
}

static void RunFocRotatingThetaVoltageTestLoop(void)
{
	float theta_step_rad;
	float control_theta;
	float vq_cmd;
	uint16_t log_samples;

	theta_step_rad =
		(2.0f * PI * ROTATING_THETA_VOLTAGE_TEST_ELECTRICAL_FREQ_HZ) /
		GetEffectiveCurrentLoopFrequency();
	log_samples = (uint16_t)ClampFloat(
		GetEffectiveCurrentLoopFrequency() * ROTATING_THETA_VOLTAGE_TEST_LOG_TIME_S,
		1.0f,
		60000.0f);

	gIdRefA = 0.0f;
	gIqRefA = 0.0f;
	gTargetSpeedRpm = 0.0f;
	gCommandedSpeedRpm = 0.0f;
	gTracePosError = 0.0f;
	sFocRotatingThetaVoltageCommand = WrapAngle(sFocRotatingThetaVoltageCommand + theta_step_rad);
	control_theta = WrapAngle(
		sFocRotatingThetaVoltageCommand + ((ROTATING_THETA_VOLTAGE_TEST_FRAME_DEG * PI) / 180.0f));
	gFocRotatingThetaVoltageDebugThetaDeg = control_theta * (180.0f / PI);
	UpdateMeasuredCurrentsForTheta(control_theta, Parameter.fIabc[0], Parameter.fIabc[1]);
	vq_cmd = ROTATING_THETA_VOLTAGE_TEST_VQ_V;
	if (Parameter.fVdc < 1.0f)
	{
		vq_cmd = 0.0f;
	}
	else if (vq_cmd > (Parameter.fVdc * 0.25f))
	{
		vq_cmd = Parameter.fVdc * 0.25f;
	}
	ApplyVoltageVectorForTheta(control_theta, 0.0f, vq_cmd);
	gFocRotatingThetaVoltageDebugDeltaPos =
		(int32_t)lroundf(Parameter.fPosition - sFocRotatingThetaVoltageStartPosition);

	if (++sFocRotatingThetaVoltageLogCounter >= log_samples)
	{
		sFocRotatingThetaVoltageLogCounter = 0u;
		USB_QueueFocDebugText(
			"[RTV] th=%.1f id=%.3f iq=%.3f iu=%.3f iv=%.3f pos=%ld sp=%.1f vq=%.2f",
			gFocRotatingThetaVoltageDebugThetaDeg,
			Parameter.fIdq[0],
			Parameter.fIdq[1],
			Parameter.fIabc[0],
			Parameter.fIabc[1],
			(long)gFocRotatingThetaVoltageDebugDeltaPos,
			Parameter.fActSpeed,
			vq_cmd);
		FOC_DEBUG_PRINTF(
			"[RTV] theta=%.1f deg id=%.3f iq=%.3f iu=%.3f iv=%.3f pos=%ld speed=%.1f rpm vq=%.2f\r\n",
			gFocRotatingThetaVoltageDebugThetaDeg,
			Parameter.fIdq[0],
			Parameter.fIdq[1],
			Parameter.fIabc[0],
			Parameter.fIabc[1],
			(long)gFocRotatingThetaVoltageDebugDeltaPos,
			Parameter.fActSpeed,
			vq_cmd);
	}
}

static void RunFocCurrentFeedbackMapTestLoop(void)
{
	float voltage_limit;
	float theta_step_rad;
	float control_theta;
	uint16_t case_samples;
	uint8_t current_case;
	uint32_t sample_count;
	float id_abs_avg;
	float iq_avg;
	float iq_err_abs_avg;
	float speed_abs_avg;
	int32_t delta_pos;
	float best_score;
	uint8_t best_case;
	uint8_t index;

	voltage_limit = Parameter.fVdc * ROTATING_THETA_TEST_VOLTAGE_LIMIT_RATIO;
	if (voltage_limit < ROTATING_THETA_TEST_MIN_VOLTAGE_LIMIT_V)
	{
		voltage_limit = ROTATING_THETA_TEST_MIN_VOLTAGE_LIMIT_V;
	}
	if (voltage_limit > ROTATING_THETA_TEST_MAX_VOLTAGE_LIMIT_V)
	{
		voltage_limit = ROTATING_THETA_TEST_MAX_VOLTAGE_LIMIT_V;
	}
	theta_step_rad =
		(2.0f * PI * ROTATING_THETA_TEST_ELECTRICAL_FREQ_HZ) /
		GetEffectiveCurrentLoopFrequency();
	case_samples = (uint16_t)ClampFloat(
		GetEffectiveCurrentLoopFrequency() * CURRENT_FEEDBACK_MAP_TEST_CASE_TIME_S,
		1.0f,
		60000.0f);

	LoadDiagnosticCurrentPiGains();
	gIdRefA = 0.0f;
	gIqRefA = ROTATING_THETA_TEST_IQ_A;
	gTargetSpeedRpm = 0.0f;
	gCommandedSpeedRpm = 0.0f;
	gTracePosError = 0.0f;
	sFocCurrentFeedbackMapCommand = WrapAngle(sFocCurrentFeedbackMapCommand + theta_step_rad);
	control_theta = WrapAngle(
		sFocCurrentFeedbackMapCommand + ((ROTATING_THETA_TEST_FRAME_DEG * PI) / 180.0f));
	UpdateMeasuredCurrentsForTheta(control_theta, Parameter.fIabc[0], Parameter.fIabc[1]);
	RunCurrentLoopForTheta(control_theta, 0.0f, ROTATING_THETA_TEST_IQ_A, voltage_limit, 0u);

	sFocCurrentFeedbackMapIdAbsAccum += fabsf(Parameter.fIdq[0]);
	sFocCurrentFeedbackMapIqAccum += Parameter.fIdq[1];
	sFocCurrentFeedbackMapIqErrAbsAccum += fabsf(ROTATING_THETA_TEST_IQ_A - Parameter.fIdq[1]);
	sFocCurrentFeedbackMapSpeedAbsAccum += fabsf(Parameter.fActSpeed);
	sFocCurrentFeedbackMapSampleCount++;

	if (++sFocCurrentFeedbackMapCounter < case_samples)
	{
		return;
	}

	current_case = sFocCurrentFeedbackMapCaseIndex;
	sample_count = (sFocCurrentFeedbackMapSampleCount > 0u) ? sFocCurrentFeedbackMapSampleCount : 1u;
	id_abs_avg = sFocCurrentFeedbackMapIdAbsAccum / (float)sample_count;
	iq_avg = sFocCurrentFeedbackMapIqAccum / (float)sample_count;
	iq_err_abs_avg = sFocCurrentFeedbackMapIqErrAbsAccum / (float)sample_count;
	speed_abs_avg = sFocCurrentFeedbackMapSpeedAbsAccum / (float)sample_count;
	delta_pos = (int32_t)lroundf(Parameter.fPosition - sFocCurrentFeedbackMapStartPosition);

	sFocCurrentFeedbackMapIdAbsAvg[current_case] = id_abs_avg;
	sFocCurrentFeedbackMapIqAvg[current_case] = iq_avg;
	sFocCurrentFeedbackMapIqErrAbsAvg[current_case] = iq_err_abs_avg;
	sFocCurrentFeedbackMapSpeedAbsAvg[current_case] = speed_abs_avg;
	sFocCurrentFeedbackMapDeltaPos[current_case] = delta_pos;

	USB_QueueFocDebugText(
		"[CFM] case=%u name=%s swap=%u inv=%u idabs=%.3f iqavg=%.3f iqerr=%.3f sp=%.1f pos=%ld",
		(unsigned int)current_case,
		GetCurrentFeedbackMapCaseName(current_case),
		(unsigned int)sFocCurrentFeedbackMapSwapCases[current_case],
		(unsigned int)sFocCurrentFeedbackMapInvertCases[current_case],
		id_abs_avg,
		iq_avg,
		iq_err_abs_avg,
		speed_abs_avg,
		(long)delta_pos);
	FOC_DEBUG_PRINTF(
		"[CFM] case=%u name=%s swap=%u inv=%u idabs=%.3f iqavg=%.3f iqerr=%.3f speed=%.1f pos=%ld\r\n",
		(unsigned int)current_case,
		GetCurrentFeedbackMapCaseName(current_case),
		(unsigned int)sFocCurrentFeedbackMapSwapCases[current_case],
		(unsigned int)sFocCurrentFeedbackMapInvertCases[current_case],
		id_abs_avg,
		iq_avg,
		iq_err_abs_avg,
		speed_abs_avg,
		(long)delta_pos);

	sFocCurrentFeedbackMapCaseIndex++;
	if (sFocCurrentFeedbackMapCaseIndex >= CURRENT_FEEDBACK_MAP_TEST_CASE_COUNT)
	{
		best_case = 0u;
		best_score = sFocCurrentFeedbackMapIdAbsAvg[0] + sFocCurrentFeedbackMapIqErrAbsAvg[0];
		for (index = 1u; index < CURRENT_FEEDBACK_MAP_TEST_CASE_COUNT; index++)
		{
			float score = sFocCurrentFeedbackMapIdAbsAvg[index] + sFocCurrentFeedbackMapIqErrAbsAvg[index];
			if (score < best_score)
			{
				best_score = score;
				best_case = index;
			}
		}
		USB_QueueFocDebugText(
			"[CFM] best=%u name=%s swap=%u inv=%u score=%.3f",
			(unsigned int)best_case,
			GetCurrentFeedbackMapCaseName(best_case),
			(unsigned int)sFocCurrentFeedbackMapSwapCases[best_case],
			(unsigned int)sFocCurrentFeedbackMapInvertCases[best_case],
			best_score);
		gFocCurrentFeedbackMapTestRunning = 0u;
		gFocCurrentUvSwapTest = 0u;
		gFocCurrentPolarityInvertTest = 0u;
		gIdRefA = 0.0f;
		gIqRefA = 0.0f;
		ApplyVoltageVectorForTheta(Parameter.fTheta, 0.0f, 0.0f);
		return;
	}

	gFocCurrentUvSwapTest = sFocCurrentFeedbackMapSwapCases[sFocCurrentFeedbackMapCaseIndex];
	gFocCurrentPolarityInvertTest = sFocCurrentFeedbackMapInvertCases[sFocCurrentFeedbackMapCaseIndex];
	sFocCurrentFeedbackMapCounter = 0u;
	sFocCurrentFeedbackMapStartPosition = Parameter.fPosition;
	sFocCurrentFeedbackMapIdAbsAccum = 0.0f;
	sFocCurrentFeedbackMapIqAccum = 0.0f;
	sFocCurrentFeedbackMapIqErrAbsAccum = 0.0f;
	sFocCurrentFeedbackMapSpeedAbsAccum = 0.0f;
	sFocCurrentFeedbackMapSampleCount = 0u;
	ResetControlLoops();
	USB_QueueFocDebugText(
		"[CFM] case=%u name=%s swap=%u inv=%u begin",
		(unsigned int)sFocCurrentFeedbackMapCaseIndex,
		GetCurrentFeedbackMapCaseName(sFocCurrentFeedbackMapCaseIndex),
		(unsigned int)gFocCurrentUvSwapTest,
		(unsigned int)gFocCurrentPolarityInvertTest);
}
static float *ResolveTraceChannelPointer(uint8_t channel_code)
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

static void UpdateTraceCapture(void)
{
	uint16_t index;

	if ((Trace_Data.Enable == 0u) || (Trace_Data.Finish != 0u))
	{
		return;
	}

	if (Trace_Data.u8CntSample > 0u)
	{
		Trace_Data.u8CntSample--;
		return;
	}

	Trace_Data.u8CntSample = Trace_Data.u8MaxCntSample;
	if (Trace_Data.Counter >= TRACE_DATA_LENGTH)
	{
		Trace_Data.Counter = 0u;
		Trace_Data.u16Cnt2 = 0u;
		Trace_Data.Finish = 1u;
		return;
	}

	index = (uint16_t)(Trace_Data.Counter * 4u);
	RecordTable1[index + 0u] = (Trace_Data.Data1 != 0) ? *Trace_Data.Data1 : 0.0f;
	RecordTable1[index + 1u] = (Trace_Data.Data2 != 0) ? *Trace_Data.Data2 : 0.0f;
	RecordTable1[index + 2u] = (Trace_Data.Data3 != 0) ? *Trace_Data.Data3 : 0.0f;
	RecordTable1[index + 3u] = (Trace_Data.Data4 != 0) ? *Trace_Data.Data4 : 0.0f;
	Trace_Data.Counter++;
}

void UpdateDriverParameter(float *driver_parameter)
{
	float max_speed;

	if (driver_parameter == 0)
	{
		return;
	}

	if (((uint8_t)driver_parameter[CONTROL_MODE] != SPEED_CONTROL_MODE) &&
		((uint8_t)driver_parameter[CONTROL_MODE] != POSITION_CONTROL_MODE))
	{
		driver_parameter[CONTROL_MODE] = (float)SPEED_CONTROL_MODE;
	}
	if ((!isfinite(driver_parameter[POSITION_TRACKING_MODE])) ||
		(((uint8_t)driver_parameter[POSITION_TRACKING_MODE]) > POSITION_TRACKING_MODE_MULTI_TURN))
	{
		driver_parameter[POSITION_TRACKING_MODE] = (float)POSITION_TRACKING_MODE_SINGLE_TURN;
	}
	if (driver_parameter[POSITION_P_GAIN] < 0.0f)
	{
		driver_parameter[POSITION_P_GAIN] = 0.05f;
	}
	if (driver_parameter[POSITION_I_GAIN] < 0.0f)
	{
		driver_parameter[POSITION_I_GAIN] = 0.50f;
	}
	if (driver_parameter[POSITION_FF_GAIN] < 0.0f)
	{
		driver_parameter[POSITION_FF_GAIN] = 0.0f;
	}
	if (driver_parameter[POSITION_FF_FILTER] <= 0.0f)
	{
		driver_parameter[POSITION_FF_FILTER] = POSITION_VFF_FILTER_DEFAULT_HZ;
	}
	if (driver_parameter[SPEED_P_GAIN] < 0.0f)
	{
		driver_parameter[SPEED_P_GAIN] = 0.02f;
	}
	if (driver_parameter[SPEED_I_GAIN] < 0.0f)
	{
		driver_parameter[SPEED_I_GAIN] = 5.0f;
	}
	if (driver_parameter[MAXIMUM_SPEED] <= 0.0f)
	{
		driver_parameter[MAXIMUM_SPEED] = 1500.0f;
	}
	if (driver_parameter[ACCELERATION_TIME] <= 0.0f)
	{
		driver_parameter[ACCELERATION_TIME] = 250.0f;
	}
	if (driver_parameter[DECELERATION_TIME] <= 0.0f)
	{
		driver_parameter[DECELERATION_TIME] = 250.0f;
	}

	max_speed = driver_parameter[MAXIMUM_SPEED];

	gSpeedPi.fDtSec = 1.0f / GetEffectiveSpeedLoopFrequency();
	gSpeedPi.fKp = driver_parameter[SPEED_P_GAIN];
	gSpeedPi.fKi = driver_parameter[SPEED_I_GAIN];
	gSpeedPi.fUpOutLim = GetConfiguredSpeedIqLimitA();
	gSpeedPi.fLowOutLim = -gSpeedPi.fUpOutLim;

	gPositionPi.fDtSec = 1.0f / GetEffectiveSpeedLoopFrequency();
	gPositionPi.fKp = driver_parameter[POSITION_P_GAIN];
	gPositionPi.fKi = GetConfiguredPositionIntegralGain();
	gPositionPi.fUpOutLim = max_speed;
	gPositionPi.fLowOutLim = -max_speed;

	gTargetSpeedRpm = ClampFloat(gTargetSpeedRpm, -max_speed, max_speed);
}

void UpdateMotorParameter(float *motor_parameter)
{
	float rated_current;
	float current_kp;
	float current_ki;

	if (motor_parameter == 0)
	{
		return;
	}

	if (motor_parameter[MOTOR_ENCODER_ID] <= 0.0f)
	{
		motor_parameter[MOTOR_ENCODER_ID] = (float)DEFAULT_MOTOR_ENCODER_ID;
	}
	if (motor_parameter[MOTOR_ENCODER_RESOLUTION] <= 0.0f)
	{
		motor_parameter[MOTOR_ENCODER_RESOLUTION] = (float)MOTOR_ENC_RES;
	}
	if (motor_parameter[MOTOR_NUMBER_POLE_PAIRS] <= 0.0f)
	{
		motor_parameter[MOTOR_NUMBER_POLE_PAIRS] = (float)INITIAL_MOTOR_POLE_PAIRS;
	}
	if (motor_parameter[MOTOR_RATED_CURRENT_RMS] <= 0.0f)
	{
		motor_parameter[MOTOR_RATED_CURRENT_RMS] = DEFAULT_MOTOR_RATED_CURRENT_RMS;
	}
	if (motor_parameter[MOTOR_PEAK_CURRENT_RMS] <= 0.0f)
	{
		motor_parameter[MOTOR_PEAK_CURRENT_RMS] = DEFAULT_MOTOR_PEAK_CURRENT_RMS;
	}
	if (motor_parameter[MOTOR_MAXIMUM_POWER] <= 0.0f)
	{
		motor_parameter[MOTOR_MAXIMUM_POWER] = DEFAULT_MOTOR_MAXIMUM_POWER_W;
	}
	if (motor_parameter[MOTOR_MAXIMUM_VOLTAGE] <= 0.0f)
	{
		motor_parameter[MOTOR_MAXIMUM_VOLTAGE] = DEFAULT_MOTOR_MAXIMUM_VOLTAGE_V;
	}
	if (motor_parameter[MOTOR_MAXIMUM_SPEED] <= 0.0f)
	{
		motor_parameter[MOTOR_MAXIMUM_SPEED] = DEFAULT_MOTOR_RATED_SPEED_RPM;
	}
	if (motor_parameter[MOTOR_CURRENT_P_GAIN] < 0.0f)
	{
		motor_parameter[MOTOR_CURRENT_P_GAIN] = 1.0f;
	}
	if (motor_parameter[MOTOR_CURRENT_I_GAIN] < 0.0f)
	{
		motor_parameter[MOTOR_CURRENT_I_GAIN] = 150.0f;
	}

	rated_current = motor_parameter[MOTOR_RATED_CURRENT_RMS];
	current_kp = motor_parameter[MOTOR_CURRENT_P_GAIN];
	current_ki = motor_parameter[MOTOR_CURRENT_I_GAIN];

	Parameter.EncRes = (uint32_t)motor_parameter[MOTOR_ENCODER_RESOLUTION];
	Parameter.u8PolePair = (uint8_t)motor_parameter[MOTOR_NUMBER_POLE_PAIRS];
	Parameter.Offset_Enc = (int32_t)motor_parameter[MOTOR_HALL_OFFSET];
	Current_Sensor.OverCurrentThreshold = ClampFloat(
		rated_current,
		0.5f,
		DIRVER_OVER_CURRENT_THRESHOLD_AMPERE_UNIT);

	gIdPi.fDtSec = 1.0f / GetEffectiveCurrentLoopFrequency();
	gIdPi.fKp = current_kp;
	gIdPi.fKi = current_ki;
	gIdPi.fUpOutLim = 0.0f;
	gIdPi.fLowOutLim = 0.0f;

	gIqPi.fDtSec = 1.0f / GetEffectiveCurrentLoopFrequency();
	gIqPi.fKp = current_kp;
	gIqPi.fKi = current_ki;
	gIqPi.fUpOutLim = 0.0f;
	gIqPi.fLowOutLim = 0.0f;

	*(__IO uint16_t*)(REG_ENCODER_ID) = (uint16_t)motor_parameter[MOTOR_ENCODER_ID];
	gFpgaEncoderParserConfigured = 0u;
	gFpgaDoneLatched = 0u;
	gFpgaDoneTickMs = HAL_GetTick();
	RefreshEncoderAlignmentPolicy();
	UpdateDriverParameter(DriverParameter);
}

static void LoadDefaultParameters(void)
{
	memset(DriverParameter, 0, sizeof(DriverParameter));
	memset(MotorParameter, 0, sizeof(MotorParameter));

	DriverParameter[CONTROL_MODE] = (float)SPEED_CONTROL_MODE;
	DriverParameter[POSITION_P_GAIN] = 0.05f;
	DriverParameter[POSITION_I_GAIN] = 0.50f;
	DriverParameter[POSITION_FF_GAIN] = 0.0f;
	DriverParameter[POSITION_FF_FILTER] = POSITION_VFF_FILTER_DEFAULT_HZ;
	DriverParameter[SPEED_P_GAIN] = 0.001f;
	DriverParameter[SPEED_I_GAIN] = 0.001f;
	DriverParameter[ACCELERATION_TIME] = 1000.0f;
	DriverParameter[DECELERATION_TIME] = 1000.0f;
	DriverParameter[MAXIMUM_SPEED] = DEFAULT_MOTOR_RATED_SPEED_RPM;
	DriverParameter[SPEED_UNIT] = 0.0f;
	DriverParameter[POSITION_TRACKING_MODE] = (float)POSITION_TRACKING_MODE_SINGLE_TURN;

	MotorParameter[MOTOR_RATED_CURRENT_RMS] = DEFAULT_MOTOR_RATED_CURRENT_RMS;
	MotorParameter[MOTOR_PEAK_CURRENT_RMS] = DEFAULT_MOTOR_PEAK_CURRENT_RMS;
	MotorParameter[MOTOR_ENCODER_ID] = (float)DEFAULT_MOTOR_ENCODER_ID;
	MotorParameter[MOTOR_ENCODER_RESOLUTION] = (float)MOTOR_ENC_RES;
	MotorParameter[MOTOR_MAXIMUM_POWER] = DEFAULT_MOTOR_MAXIMUM_POWER_W;
	MotorParameter[MOTOR_MAXIMUM_VOLTAGE] = DEFAULT_MOTOR_MAXIMUM_VOLTAGE_V;
	MotorParameter[MOTOR_MAXIMUM_SPEED] = DEFAULT_MOTOR_RATED_SPEED_RPM;
	MotorParameter[MOTOR_NUMBER_POLE_PAIRS] = (float)INITIAL_MOTOR_POLE_PAIRS;
	MotorParameter[MOTOR_CURRENT_P_GAIN] = 3.0f;
	MotorParameter[MOTOR_CURRENT_I_GAIN] = 5.0f;
	MotorParameter[MOTOR_CURRENT_CTRL_DIRECTION] = 0.0f;
	IdSquareTuning.Mode = ID_SQUARE_TUNING_MODE_SQUARE_WAVE;
	gEncoderAlignmentStatus = ENCODER_ALIGNMENT_STATUS_IDLE;
	gEncoderAlignmentNeedsFlashSave = 0u;
	gEncoderAlignmentLastCapturedOffset = 0;

	UpdateMotorParameter(MotorParameter);
	UpdateDriverParameter(DriverParameter);
}

static void ReadFastProtectionFeedback(void)
{
	gRawVdc = *(__IO uint16_t*)(REG_DC_BUS_VOLTAGE);
	gRawCurrentU = *(__IO uint16_t*)(REG_CURRENT_PHASE_U);
	gRawCurrentV = *(__IO uint16_t*)(REG_CURRENT_PHASE_V);
	gRawTemp = *(__IO uint16_t*)(REG_TEMPARATURE_SENSOR);

	Parameter.fVdc = (float)((int32_t)gRawVdc - OFFSET) / Resolution16bits * INPUT_RANGE_VDC;
	Parameter.fIabc[0] = -(float)((int32_t)gRawCurrentU - (int32_t)Parameter.u16Offset_Ia) / Resolution16bits * INPUT_RANGE_I;
	Parameter.fIabc[1] = -(float)((int32_t)gRawCurrentV - (int32_t)Parameter.u16Offset_Ib) / Resolution16bits * INPUT_RANGE_I;
	Parameter.fIabc[2] = -Parameter.fIabc[0] - Parameter.fIabc[1];
	Parameter.fTemparature = (float)((int32_t)gRawTemp - OFFSET) / Resolution16bits * INPUT_RANGE_TEMPARATURE;
}

static void UpdateMeasuredSpeedAndTheta(void)
{
	float encoder_resolution;
	float electrical_angle;
	float mechanical_angle;
	float raw_speed_rpm;
	float position_single_turn;

	encoder_resolution = (MotorParameter[MOTOR_ENCODER_RESOLUTION] > 0.0f) ?
		MotorParameter[MOTOR_ENCODER_RESOLUTION] : (float)MOTOR_ENC_RES;

	raw_speed_rpm = ((float)Parameter.DeltaPos * GetEffectiveCurrentLoopFrequency() * 60.0f) / encoder_resolution;
	
	if (MotorParameter[MOTOR_CURRENT_CTRL_DIRECTION] == 0.0f)
	{
			raw_speed_rpm = -raw_speed_rpm;
	}
	
	gDebugSpeedRawRpm = raw_speed_rpm;
	Parameter.fActSpeedFilter += SPEED_ESTIMATE_LPF_ALPHA * (raw_speed_rpm - Parameter.fActSpeedFilter);
	Parameter.fActSpeed = Parameter.fActSpeedFilter;
	gDebugObservedElectricalHz = fabsf(Parameter.fActSpeed) * ((float)Parameter.u8PolePair / 60.0f);
	gDebugEncoderTurns = Parameter.fPosition / encoder_resolution;

	sDebugDeltaPosAccum += (float)Parameter.DeltaPos;
	sDebugSpeedRawAccum += raw_speed_rpm;
	sDebugObservedElecHzAccum += gDebugObservedElectricalHz;
	sDebugAvgCounter++;
	if (sDebugAvgCounter >= DEBUG_AVG_SAMPLES)
	{
		gDebugDeltaPosAvg = sDebugDeltaPosAccum / (float)sDebugAvgCounter;
		gDebugSpeedRawRpmAvg = sDebugSpeedRawAccum / (float)sDebugAvgCounter;
		gDebugObservedElectricalHzAvg = sDebugObservedElecHzAccum / (float)sDebugAvgCounter;
		sDebugDeltaPosAccum = 0.0f;
		sDebugSpeedRawAccum = 0.0f;
		sDebugObservedElecHzAccum = 0.0f;
		sDebugAvgCounter = 0u;
	}

	if (MotorParameter[MOTOR_CURRENT_CTRL_DIRECTION] == 0.0f)
	{
		position_single_turn = (float)Parameter.EncSingleTurn;
	}
	else
	{
		position_single_turn = encoder_resolution - (float)Parameter.EncSingleTurn;
	}

	position_single_turn += (float)Parameter.Offset_Enc;
	position_single_turn = fmodf(position_single_turn, encoder_resolution);
	if (position_single_turn < 0.0f)
	{
		position_single_turn += encoder_resolution;
	}
	// Invert for position theta 0 to 2pi match Lib vector tranform
  mechanical_angle = (1.0f - (position_single_turn / encoder_resolution)) * (2.0f * PI);
	// mechanical_angle = (position_single_turn / encoder_resolution) * (2.0f * PI);
	electrical_angle = mechanical_angle * (float)Parameter.u8PolePair;
	gDebugMechanicalAngleRad = WrapAngle(mechanical_angle);
	gDebugElectricalAngleRad = WrapAngle(electrical_angle);
	Parameter.fTheta = WrapAngle(electrical_angle);
}

static void UpdateMeasuredCurrentsForTheta(float electrical_theta, float phase_u, float phase_v)
{
	float sin_theta = sinf(electrical_theta);
	float cos_theta = cosf(electrical_theta);

	ApplyPhaseCurrentFeedbackMapping(&phase_u, &phase_v);

	gClarke.fA = phase_u;
	gClarke.fB = phase_v;
	gClarke.m_ab2albe(&gClarke);

	gPark.fAl = gClarke.fAl;
	gPark.fBe = gClarke.fBe;
	gPark.fSinAng = sin_theta;
	gPark.fCosAng = cos_theta;
	gPark.m_albe2dq(&gPark);

	Parameter.fIdq[0] = gPark.fD;
	Parameter.fIdq[1] = gPark.fQ;
}

static void ApplyPhaseCurrentFeedbackMapping(float *phase_u, float *phase_v)
{
	float current_temp;
	uint8_t apply_swap;
	uint8_t apply_invert;

	if ((phase_u == 0) || (phase_v == 0))
	{
		return;
	}

	/* Current polarity is a driver hardware characteristic for this board.
	   Keep it forced in firmware so every motor uses the same validated
	   feedback sign. U/V swap remains available only as an extra debug aid. */
	if (IdSquareTuning.Enable != 0u)
	{
		apply_swap = (FORCE_DRIVER_CURRENT_UV_SWAP != 0u) || (IdSquareTuning.CurrentUvSwapTest != 0u);
		apply_invert = (FORCE_DRIVER_CURRENT_POLARITY_INVERT != 0u) || (IdSquareTuning.CurrentPolarityInvertTest != 0u);
	}
	else
	{
		apply_swap = (FORCE_DRIVER_CURRENT_UV_SWAP != 0u) || (gFocCurrentUvSwapTest != 0u);
		apply_invert = (FORCE_DRIVER_CURRENT_POLARITY_INVERT != 0u) || (gFocCurrentPolarityInvertTest != 0u);
	}

	if (apply_swap != 0u)
	{
		current_temp = *phase_u;
		*phase_u = *phase_v;
		*phase_v = current_temp;
	}
	if (apply_invert != 0u)
	{
		*phase_u = -*phase_u;
		*phase_v = -*phase_v;
	}
}

static void ApplyVoltageVectorForTheta(float electrical_theta, float vd, float vq)
{
	float sin_theta;
	float cos_theta;
	float inv_vbus;
	float duty_u;
	float duty_v;
	float duty_w;

	Parameter.fVdq[0] = vd;
	Parameter.fVdq[1] = vq;

	if (Parameter.fVdc < 1.0f)
	{
		Parameter.fVabc[0] = 0.0f;
		Parameter.fVabc[1] = 0.0f;
		Parameter.fVabc[2] = 0.0f;
		GeneratePWM(0.5f, 0.5f, 0.5f);
		return;
	}

	sin_theta = sinf(electrical_theta);
	cos_theta = cosf(electrical_theta);

	gInvPark.fD = vd;
	gInvPark.fQ = vq;
	gInvPark.fSinAng = sin_theta;
	gInvPark.fCosAng = cos_theta;
	gInvPark.m_dq2albe(&gInvPark);

	gInvClarke.fAl = gInvPark.fAl;
	gInvClarke.fBe = gInvPark.fBe;
	gInvClarke.m_albe2abc(&gInvClarke);

	Parameter.fVabc[0] = gInvClarke.fA;
	Parameter.fVabc[1] = gInvClarke.fB;
	Parameter.fVabc[2] = gInvClarke.fC;

	inv_vbus = 1.0f / Parameter.fVdc;
	duty_u = 0.5f + (Parameter.fVabc[0] * inv_vbus);
	duty_v = 0.5f + (Parameter.fVabc[1] * inv_vbus);
	duty_w = 0.5f + (Parameter.fVabc[2] * inv_vbus);

	GeneratePWM(
		ClampFloat(duty_u, 0.05f, 0.95f),
		ClampFloat(duty_v, 0.05f, 0.95f),
		ClampFloat(duty_w, 0.05f, 0.95f));
}

static void RunCurrentLoopForTheta(
	float electrical_theta,
	float id_ref,
	float iq_ref,
	float voltage_limit,
	uint8_t isolate_q_axis)
{
	float vd_command;
	float vq_command;

	if (voltage_limit < 0.5f)
	{
		voltage_limit = 0.5f;
	}

	gIdPi.fUpOutLim = voltage_limit;
	gIdPi.fLowOutLim = -voltage_limit;
	gIqPi.fUpOutLim = voltage_limit;
	gIqPi.fLowOutLim = -voltage_limit;

	if (ShouldHoldCurrentLoopAtZero(id_ref, Parameter.fIdq[0]) != 0u)
	{
		gIdPi.m_rst(&gIdPi);
		gIdPi.fIn = 0.0f;
		vd_command = 0.0f;
	}
	else
	{
		gIdPi.fIn = id_ref - Parameter.fIdq[0];
		gIdPi.m_calc(&gIdPi);
		vd_command = gIdPi.fOut;
	}

	if (isolate_q_axis != 0u)
	{
		gIqPi.m_rst(&gIqPi);
		gIqPi.fIn = 0.0f;
		vq_command = 0.0f;
	}
	else
	{
		if (ShouldHoldCurrentLoopAtZero(iq_ref, Parameter.fIdq[1]) != 0u)
		{
			gIqPi.m_rst(&gIqPi);
			gIqPi.fIn = 0.0f;
			vq_command = 0.0f;
		}
		else
		{
			gIqPi.fIn = iq_ref - Parameter.fIdq[1];
			gIqPi.m_calc(&gIqPi);
			vq_command = gIqPi.fOut;
		}
		LimitDqVoltageVector(&vd_command, &vq_command, voltage_limit);
	}

	ApplyVoltageVectorForTheta(electrical_theta, vd_command, vq_command);
}

static float GetOpenLoopVoltageLimit(void)
{
	float voltage_limit = Parameter.fVdc * 0.7f;
	float motor_voltage_limit = MotorParameter[MOTOR_MAXIMUM_VOLTAGE];

	if ((motor_voltage_limit > 0.0f) && (motor_voltage_limit < voltage_limit))
	{
		voltage_limit = motor_voltage_limit;
	}
	if (voltage_limit < 6.0f)
	{
		voltage_limit = 6.0f;
	}
	if (voltage_limit > 40.0f)
	{
		voltage_limit = 40.0f;
	}
	return voltage_limit;
}

static float GetOpenLoopFrequencyLimitHz(void)
{
	float max_speed_rpm = MotorParameter[MOTOR_MAXIMUM_SPEED];
	float pole_pairs = MotorParameter[MOTOR_NUMBER_POLE_PAIRS];

	if (max_speed_rpm <= 0.0f)
	{
		max_speed_rpm = DEFAULT_MOTOR_RATED_SPEED_RPM;
	}
	if (pole_pairs <= 0.0f)
	{
		pole_pairs = (float)INITIAL_MOTOR_POLE_PAIRS;
	}

	return (max_speed_rpm * pole_pairs) / 60.0f;
}

static void ApplyOpenLoopVfCommand(float requested_frequency, float requested_voltage)
{
	float frequency_limit_hz = GetOpenLoopFrequencyLimitHz();
	float limited_frequency;
	float limited_voltage = ClampFloat(requested_voltage, 0.0f, GetOpenLoopVoltageLimit());
	float modulation = 0.0f;
	float pole_pairs = (Parameter.u8PolePair > 0u) ? (float)Parameter.u8PolePair : 1.0f;
	float sin_theta;
	float cos_theta;
	PWM_Phases_t vf_output;

	if (frequency_limit_hz < 1.0f)
	{
		frequency_limit_hz = DEFAULT_MOTOR_RATED_ELECTRICAL_FREQUENCY_HZ;
	}
	gOpenLoopFrequencyLimitHz = frequency_limit_hz;
	limited_frequency = ClampFloat(requested_frequency, -frequency_limit_hz, frequency_limit_hz);

	if (Parameter.fVdc > 1.0f)
	{
		modulation = limited_voltage / Parameter.fVdc;
	}
	modulation = ClampFloat(modulation, 0.0f, 0.85f);

	gIdRefA = 0.0f;
	gIqRefA = 0.0f;
	gTargetSpeedRpm = (limited_frequency * 60.0f) / pole_pairs;
	gCommandedSpeedRpm = gTargetSpeedRpm;
	gTracePosError = 0.0f;
	gDebugOpenLoopElectricalHzCmd = limited_frequency;
	gDebugOpenLoopSyncRpmCmd = gTargetSpeedRpm;
	gDebugExpectedDeltaPosSync = (limited_frequency * (MotorParameter[MOTOR_ENCODER_RESOLUTION] > 0.0f ?
		MotorParameter[MOTOR_ENCODER_RESOLUTION] : (float)MOTOR_ENC_RES)) /
		(GetEffectiveCurrentLoopFrequency() * pole_pairs);
	vf_output = Control_V_over_F(limited_frequency, modulation);

	Parameter.fVabc[0] = (vf_output.U - 0.5f) * Parameter.fVdc;
	Parameter.fVabc[1] = (vf_output.V - 0.5f) * Parameter.fVdc;
	Parameter.fVabc[2] = (vf_output.W - 0.5f) * Parameter.fVdc;

	sin_theta = sinf(Parameter.fTheta);
	cos_theta = cosf(Parameter.fTheta);

	{
		float phase_u = Parameter.fIabc[0];
		float phase_v = Parameter.fIabc[1];
		ApplyPhaseCurrentFeedbackMapping(&phase_u, &phase_v);
		gClarke.fA = phase_u;
		gClarke.fB = phase_v;
	}
	gClarke.m_ab2albe(&gClarke);

	gPark.fAl = gClarke.fAl;
	gPark.fBe = gClarke.fBe;
	gPark.fSinAng = sin_theta;
	gPark.fCosAng = cos_theta;
	gPark.m_albe2dq(&gPark);
	Parameter.fIdq[0] = gPark.fD;
	Parameter.fIdq[1] = gPark.fQ;

	gClarke.fA = Parameter.fVabc[0];
	gClarke.fB = Parameter.fVabc[1];
	gClarke.m_ab2albe(&gClarke);

	gPark.fAl = gClarke.fAl;
	gPark.fBe = gClarke.fBe;
	gPark.fSinAng = sin_theta;
	gPark.fCosAng = cos_theta;
	gPark.m_albe2dq(&gPark);
	Parameter.fVdq[0] = gPark.fD;
	Parameter.fVdq[1] = gPark.fQ;
}

static void RunOpenLoopVf(void)
{
	ApplyOpenLoopVfCommand(gVfFrequencyHz, gVfVoltageV);
}

static void ReportFault(uint16_t fault)
{
	FaultCode |= fault;
	gDebugFaultSnapshot = FaultCode;
	gFaultPhaseU = Parameter.fIabc[0];
	gFaultPhaseV = Parameter.fIabc[1];
	gFaultPhaseW = Parameter.fIabc[2];
	USB_Comm.SendError = true;
	if (gRunMode == RUN_MODE_AUTOTUNE)
	{
		if ((fault & ERROR_OVER_CURRENT) != 0u)
		{
			MotorAutoTune_SetError(&gMotorAutoTune, MOTOR_AUTOTUNE_ERROR_OVERCURRENT);
		}
		else
		{
			MotorAutoTune_SetError(&gMotorAutoTune, MOTOR_AUTOTUNE_ERROR_SIGNAL);
		}
	}
	gRunMode = RUN_MODE_FOC;
	gVfFrequencyHz = 0.0f;
	gVfVoltageV = 0.0f;
	gTargetPositionCounts = Parameter.fPosition;
	gCommandedSpeedRpm = 0.0f;
	gTracePosError = 0.0f;
	RestoreIdSquareTuningOffsetGuard();
	IdSquareTuning.Enable = 0u;
	IdSquareTuning.AlignmentDone = 0u;
	IdSquareTuning.AlignmentCounter = 0u;
	IdSquareTuning.fPhase = 0.0f;
	IdSquareTuning.fVoltageLimitApplied = 0.0f;
	IdSquareTuning.fAlignmentCurrentApplied = 0.0f;
	IdSquareTuning.OffsetCaptured = 0;
	gEncoderAlignmentRequested = 0u;
	gEncoderAlignmentContinueToRun = 0u;
	gEncoderAlignmentResumeRunMode = RUN_MODE_FOC;
	if ((gEncoderAlignmentStatus == ENCODER_ALIGNMENT_STATUS_REQUESTED) ||
		(gEncoderAlignmentStatus == ENCODER_ALIGNMENT_STATUS_RUNNING))
	{
		gEncoderAlignmentStatus = ENCODER_ALIGNMENT_STATUS_FAULT;
	}
	if (gFocRotatingThetaTestRunning != 0u)
	{
		USB_QueueFocDebugText(
			"[RTH] fault=0x%04X th=%.1f off=%ld",
			(unsigned int)fault,
			gFocRotatingThetaDebugThetaDeg,
			(long)Parameter.Offset_Enc);
		gFocRotatingThetaTestRunning = 0u;
	}
	if (gFocRotatingThetaVoltageTestRunning != 0u)
	{
		USB_QueueFocDebugText(
			"[RTV] fault=0x%04X th=%.1f off=%ld",
			(unsigned int)fault,
			gFocRotatingThetaVoltageDebugThetaDeg,
			(long)Parameter.Offset_Enc);
		gFocRotatingThetaVoltageTestRunning = 0u;
	}
	if (gFocCurrentFeedbackMapTestRunning != 0u)
	{
		USB_QueueFocDebugText(
			"[CFM] fault=0x%04X case=%u off=%ld",
			(unsigned int)fault,
			(unsigned int)sFocCurrentFeedbackMapCaseIndex,
			(long)Parameter.Offset_Enc);
		gFocCurrentFeedbackMapTestRunning = 0u;
		gFocCurrentUvSwapTest = 0u;
		gFocCurrentPolarityInvertTest = 0u;
	}
	STM_FaultProcessing(&StateMachine, fault, 0xFFFFFFFFu);
	ResetControlLoops();
	SetPwmEnabled(0u);
	system_on = 0u;
}
static void RunFocLoop(void)
{
	float sin_theta;
	float cos_theta;
	float voltage_limit;
	float inv_vbus;
	float duty_u;
	float duty_v;
	float duty_w;
	float control_theta;
	float current_phase_u;
	float current_phase_v;
	float encoder_resolution;
	uint8_t alignment_active = 0u;
	uint8_t alignment_hold_mode = 0u;

	if (Parameter.fVdc < 1.0f)
	{
		GeneratePWM(0.5f, 0.5f, 0.5f);
		return;
	}

	if (gFocRotatingThetaTestRunning != 0u)
	{
		RunFocRotatingThetaTestLoop();
		return;
	}
	if (gFocRotatingThetaVoltageTestRunning != 0u)
	{
		RunFocRotatingThetaVoltageTestLoop();
		return;
	}
	if (gFocCurrentFeedbackMapTestRunning != 0u)
	{
		RunFocCurrentFeedbackMapTestLoop();
		return;
	}

	encoder_resolution = (MotorParameter[MOTOR_ENCODER_RESOLUTION] > 0.0f) ?
		MotorParameter[MOTOR_ENCODER_RESOLUTION] : (float)MOTOR_ENC_RES;
	if ((IdSquareTuning.Enable != 0u) &&
		(IdSquareTuning.Mode == ID_SQUARE_TUNING_MODE_SQUARE_WAVE))
	{
		if (sIdSquareTuningOffsetGuardActive == 0u)
		{
			sIdSquareTuningSavedOffset = Parameter.Offset_Enc;
			sIdSquareTuningOffsetGuardActive = 1u;
			FOC_DEBUG_PRINTF(
				"[IDTUNE] guard save offset=%ld\r\n",
				(long)sIdSquareTuningSavedOffset);
		}
	}
	else
	{
		RestoreIdSquareTuningOffsetGuard();
	}

	alignment_active = ((IdSquareTuning.Enable != 0u) &&
		(IdSquareTuning.Mode == ID_SQUARE_TUNING_MODE_ALIGNMENT_HOLD) &&
		(IdSquareTuning.AlignmentDone == 0u)) ? 1u : 0u;
	alignment_hold_mode = ((IdSquareTuning.Enable != 0u) &&
		(IdSquareTuning.Mode == ID_SQUARE_TUNING_MODE_ALIGNMENT_HOLD) &&
		(IdSquareTuning.AlignmentDone != 0u)) ? 1u : 0u;

	if (IdSquareTuning.Enable != 0u)
	{
		gIdPi.fKp = IdSquareTuning.fCurrentKpCmd;
		gIdPi.fKi = IdSquareTuning.fCurrentKiCmd;
		gIqPi.fKp = IdSquareTuning.fCurrentKpCmd;
		gIqPi.fKi = IdSquareTuning.fCurrentKiCmd;
	}
	else
	{
		gIdPi.fKp = MotorParameter[MOTOR_CURRENT_P_GAIN];
		gIdPi.fKi = MotorParameter[MOTOR_CURRENT_I_GAIN];
		gIqPi.fKp = MotorParameter[MOTOR_CURRENT_P_GAIN];
		gIqPi.fKi = MotorParameter[MOTOR_CURRENT_I_GAIN];
	}

	control_theta = Parameter.fTheta;
	
	if (alignment_active != 0u)
	{
		control_theta = 0.0f;
	}
	else if ((IdSquareTuning.Enable != 0u) &&
		(IdSquareTuning.Mode == ID_SQUARE_TUNING_MODE_SQUARE_WAVE))
	{
		// Id square-wave commissioning must stay on a fixed d-axis frame so the
		// current response is repeatable between runs and does not depend on the
		// live encoder angle at the instant the user presses Start. The
		// commissioning UI documents this as ""Forced = 0 rad"", so keep the test
		// locked to that electrical frame.
		control_theta = GetIdSquareLockedControlTheta();
	}
	else if (IdSquareTuning.Enable != 0u)
	{
		switch (IdSquareTuning.ElectricalAngleTestMode)
		{
			case ID_SQUARE_ANGLE_TEST_PLUS_90:
				control_theta = WrapAngle(control_theta + (0.5f * PI));
				break;
			case ID_SQUARE_ANGLE_TEST_MINUS_90:
				control_theta = WrapAngle(control_theta - (0.5f * PI));
				break;
			case ID_SQUARE_ANGLE_TEST_PLUS_180:
				control_theta = WrapAngle(control_theta + PI);
				break;
			case ID_SQUARE_ANGLE_TEST_NONE:
			default:
				break;
		}
	}
	else
	{
		control_theta = GetRuntimeFocControlTheta();
		switch (gFocElectricalAngleTestMode)
		{
			case ID_SQUARE_ANGLE_TEST_PLUS_90:
				control_theta = WrapAngle(control_theta + (0.5f * PI));
				break;
			case ID_SQUARE_ANGLE_TEST_MINUS_90:
				control_theta = WrapAngle(control_theta - (0.5f * PI));
				break;
			case ID_SQUARE_ANGLE_TEST_PLUS_180:
				control_theta = WrapAngle(control_theta + PI);
				break;
			case ID_SQUARE_ANGLE_TEST_NONE:
			default:
				break;
		}
	}

	// Calculate sin/cos theta
	sin_theta = sinf(control_theta);
	cos_theta = cosf(control_theta);

	current_phase_u = Parameter.fIabc[0];
	current_phase_v = Parameter.fIabc[1];
	ApplyPhaseCurrentFeedbackMapping(&current_phase_u, &current_phase_v);
	
	// Run Park Clarke conversion
	
	gClarke.fA = current_phase_u;
	gClarke.fB = current_phase_v;
	gClarke.m_ab2albe(&gClarke);

	gPark.fAl = gClarke.fAl;
	gPark.fBe = gClarke.fBe;
	gPark.fSinAng = sin_theta;
	gPark.fCosAng = cos_theta;
	gPark.m_albe2dq(&gPark);

	Parameter.fIdq[0] = gPark.fD;
	Parameter.fIdq[1] = gPark.fQ;

	// This section divide to 2 branch --> Tunning MODE & Normal Operation MODE
	if (IdSquareTuning.Enable != 0u)
	{
		gTargetSpeedRpm = 0.0f;
		gCommandedSpeedRpm = 0.0f;
		gTracePosError = 0.0f;
		gIqRefA = 0.0f;
		if ((alignment_active != 0u) || (alignment_hold_mode != 0u))
		{
			// Clamp safety for encoder alignment (inject Id to align zero point)
			gIdRefA = CalcIdSquareTuningAlignmentCurrent();
			IdSquareTuning.fFrequencyApplied = 0.0f;
			IdSquareTuning.fPhase = 0.0f;
		}
			// If align zero point of encoder is DONE --> run tunning Id square wave
		else
		{
			gIdRefA = CalcIdSquareTuningReference();
		}
		IdSquareTuning.fVoltageLimitApplied = CalcIdSquareTuningVoltageLimit();
		voltage_limit = IdSquareTuning.fVoltageLimitApplied;
	}
	else
	{
		IdSquareTuning.fVoltageLimitApplied = 0.0f;
		IdSquareTuning.fAlignmentCurrentApplied = 0.0f;
		
		// Divide speed frequency control by 2 (e.g: current loop is 16kHz --> enter interupt loop 2 times --> start calculate speed
		gSpeedLoopDivider++;
		if (gSpeedLoopDivider >= 2u)
		{
			float speed_reference_rpm = gTargetSpeedRpm;
			gSpeedLoopDivider = 0u;
			if (GetActiveFocControlMode() == POSITION_CONTROL_MODE)
			{
				float position_raw_error_counts = GetPositionControlErrorCounts(
					gTargetPositionCounts,
					Parameter.fPosition,
					encoder_resolution);
				float position_error_counts = position_raw_error_counts;
				float position_deadband_counts = GetPositionLoopErrorDeadbandCounts(encoder_resolution);
				float position_release_deadband_counts = GetPositionLoopErrorReleaseDeadbandCounts(encoder_resolution);
				float speed_limit_rpm = GetConfiguredSpeedLimitRpm();
				float setpoint_velocity_rpm = UpdatePositionSetpointVelocityRpm(
					gTargetPositionCounts,
					encoder_resolution,
					gSpeedPi.fDtSec);
				float position_vff_rpm = -GetConfiguredPositionVffGain() * setpoint_velocity_rpm;
				float position_pi_limit_rpm;
				float position_pi_output_rpm;
				float position_speed_target_rpm;
				uint8_t position_deadband_hold_active = 0u;
				uint8_t setpoint_velocity_is_quiet =
					(fabsf(setpoint_velocity_rpm) < POSITION_LOOP_FF_DEADBAND_RPM) ? 1u : 0u;

				if ((sPositionDeadbandHoldActive != 0u) &&
					((setpoint_velocity_is_quiet == 0u) ||
					 (fabsf(position_raw_error_counts) > position_release_deadband_counts)))
				{
					sPositionDeadbandHoldActive = 0u;
				}
				if ((sPositionDeadbandHoldActive == 0u) &&
					(setpoint_velocity_is_quiet != 0u) &&
					(fabsf(position_raw_error_counts) <= position_deadband_counts))
				{
					sPositionDeadbandHoldActive = 1u;
				}
				if (sPositionDeadbandHoldActive != 0u)
				{
					gPositionPi.m_rst(&gPositionPi);
					gSpeedPi.m_rst(&gSpeedPi);
					position_error_counts = 0.0f;
					gTracePosError = 0.0f;
					position_vff_rpm = 0.0f;
					position_speed_target_rpm = 0.0f;
					speed_reference_rpm = 0.0f;
					gCommandedSpeedRpm = 0.0f;
					position_deadband_hold_active = 1u;
				}
				else
				{
					gTracePosError = position_error_counts;
					position_vff_rpm = ClampFloat(
						position_vff_rpm,
						-speed_limit_rpm,
						speed_limit_rpm);
					position_pi_limit_rpm = speed_limit_rpm - fabsf(position_vff_rpm);
					if (position_pi_limit_rpm < 0.0f)
					{
						position_pi_limit_rpm = 0.0f;
					}
					gPositionPi.fDtSec = gSpeedPi.fDtSec;
					gPositionPi.fKp = GetConfiguredPositionGain();
					gPositionPi.fKi = GetConfiguredPositionIntegralGain();
					gPositionPi.fUpOutLim = position_pi_limit_rpm;
					gPositionPi.fLowOutLim = -position_pi_limit_rpm;
					gPositionPi.fIn = position_error_counts;
					gPositionPi.m_calc(&gPositionPi);
					position_pi_output_rpm = -gPositionPi.fOut;
					position_speed_target_rpm = position_pi_output_rpm + position_vff_rpm;
				}
				position_speed_target_rpm = ClampFloat(
					position_speed_target_rpm,
					-speed_limit_rpm,
					speed_limit_rpm);
				if (position_deadband_hold_active == 0u)
				{
					speed_reference_rpm = ApplySpeedRampLimit(
						gCommandedSpeedRpm,
						position_speed_target_rpm,
						speed_limit_rpm,
						gSpeedPi.fDtSec);
				}
			}
			else
			{
				float speed_limit_rpm = GetConfiguredSpeedLimitRpm();
				float target_speed_rpm;
				gTracePosError = 0.0f;
				target_speed_rpm = ClampFloat(
					gTargetSpeedRpm,
					-speed_limit_rpm,
					speed_limit_rpm);
				speed_reference_rpm = ApplySpeedRampLimit(
					gCommandedSpeedRpm,
					target_speed_rpm,
					speed_limit_rpm,
					gSpeedPi.fDtSec);
			}

			gCommandedSpeedRpm = speed_reference_rpm;
			gSpeedPi.fIn = speed_reference_rpm - Parameter.fActSpeed;
			gSpeedPi.m_calc(&gSpeedPi);
			gIqRefA = -ClampFloat(gSpeedPi.fOut, gSpeedPi.fLowOutLim, gSpeedPi.fUpOutLim); //Get Iq ref for current loop
		}

		voltage_limit = Parameter.fVdc * 0.45f;
	}

	gIdPi.fUpOutLim = voltage_limit;
	gIdPi.fLowOutLim = -voltage_limit;
	gIqPi.fUpOutLim = voltage_limit;
	gIqPi.fLowOutLim = -voltage_limit;

	
	// D loop current control
	if (ShouldHoldCurrentLoopAtZero(gIdRefA, gPark.fD) != 0u)
	{
		gIdPi.m_rst(&gIdPi);
		gIdPi.fIn = 0.0f;
		gIdPi.fOut = 0.0f;
	}
	else
	{
		gIdPi.fIn = gIdRefA - gPark.fD;
		gIdPi.m_calc(&gIdPi);
	}
	
	// If in tunning mode --> isolate and reset PI control Q - axis
	if (IdSquareTuning.Enable != 0u)
	{
		/* Keep the Id square-wave / alignment experiment on the d-axis only.
		   The q-axis PI is reset and forced to zero so Vq/Iq coupling does not
		   pollute the locked-rotor response while we validate alignment or tune Id. */
		gIqPi.m_rst(&gIqPi);
		gIqPi.fIn = 0.0f;
		gIqPi.fOut = 0.0f;
		gIqPi.fPout = 0.0f;
		gIdPi.fOut = ClampFloat(gIdPi.fOut, -voltage_limit, voltage_limit);
	}
	// If not in Id tuning mode --> calculate PI control Q - axis
	// Q loop current control
	else
	{
		if (ShouldHoldCurrentLoopAtZero(gIqRefA, gPark.fQ) != 0u)
		{
			gIqPi.m_rst(&gIqPi);
			gIqPi.fIn = 0.0f;
			gIqPi.fOut = 0.0f;
		}
		else
		{
			gIqPi.fIn = gIqRefA - gPark.fQ;
			gIqPi.m_calc(&gIqPi);
		}
		LimitDqVoltageVector(&gIdPi.fOut, &gIqPi.fOut, voltage_limit);
	}
	
	// Inverse convert ==> DQ to A B C phase supply to motor
	Parameter.fVdq[0] = gIdPi.fOut;
	Parameter.fVdq[1] = gIqPi.fOut;

	gInvPark.fD = gIdPi.fOut;
	gInvPark.fQ = gIqPi.fOut;
	gInvPark.fSinAng = sin_theta;
	gInvPark.fCosAng = cos_theta;
	gInvPark.m_dq2albe(&gInvPark); // Vd, Vq -> V_alpha, V_beta

	gInvClarke.fAl = gInvPark.fAl;
	gInvClarke.fBe = gInvPark.fBe;
	gInvClarke.m_albe2abc(&gInvClarke); // V_alpha, V_beta -> V_u, V_v, V_w

	Parameter.fVabc[0] = gInvClarke.fA;
	Parameter.fVabc[1] = gInvClarke.fB;
	Parameter.fVabc[2] = gInvClarke.fC;

	inv_vbus = 1.0f / Parameter.fVdc;
	duty_u = 0.5f + (Parameter.fVabc[0] * inv_vbus);
	duty_v = 0.5f + (Parameter.fVabc[1] * inv_vbus);
	duty_w = 0.5f + (Parameter.fVabc[2] * inv_vbus);

	GeneratePWM(
		ClampFloat(duty_u, 0.05f, 0.95f),
		ClampFloat(duty_v, 0.05f, 0.95f),
		ClampFloat(duty_w, 0.05f, 0.95f));

	if (alignment_active != 0u)
	{
		if (IdSquareTuning.AlignmentCounter <
			(uint16_t)(GetEffectiveCurrentLoopFrequency() * ID_SQUARE_TUNING_ALIGN_TIME_S))
		{
			IdSquareTuning.AlignmentCounter++;
		}
		else if (sEncoderAlignSampleCount < ENCODER_ALIGNMENT_AVG_SAMPLES)
		{
			AccumulateEncoderAlignmentSample(encoder_resolution);
		}
		else
		{
			IdSquareTuning.OffsetCaptured = CalcAveragedAlignedEncoderOffset(encoder_resolution);
			Parameter.Offset_Enc = IdSquareTuning.OffsetCaptured;
			MotorParameter[MOTOR_HALL_OFFSET] = (float)IdSquareTuning.OffsetCaptured;
			IdSquareTuning.AlignmentDone = 1u;
			IdSquareTuning.AlignmentCounter = 0u;
			ResetEncoderAlignmentAveraging();
			ResetControlLoops();
		}
	}
}

static void RunMotorAutoTuneLoop(void)
{
	MotorAutoTuneInputs_t inputs;
	MotorAutoTuneOutputs_t outputs;
	float electrical_theta = GetAutoTuneControlTheta();
	float voltage_limit;
	float pole_pairs;

	if (Parameter.fVdc < 1.0f)
	{
		GeneratePWM(0.5f, 0.5f, 0.5f);
		return;
	}

	UpdateMeasuredCurrentsForTheta(electrical_theta, Parameter.fIabc[0], Parameter.fIabc[1]);

	memset(&inputs, 0, sizeof(inputs));
	inputs.id_current_a = Parameter.fIdq[0];
	inputs.iq_current_a = Parameter.fIdq[1];
	inputs.vd_voltage_v = Parameter.fVdq[0];
	inputs.vq_voltage_v = Parameter.fVdq[1];
	inputs.phase_voltage_u_v = Parameter.fVabc[0];
	inputs.phase_voltage_v_v = Parameter.fVabc[1];
	inputs.phase_voltage_w_v = Parameter.fVabc[2];
	inputs.phase_u_a = Parameter.fIabc[0];
	inputs.phase_v_a = Parameter.fIabc[1];
	inputs.phase_w_a = Parameter.fIabc[2];
	inputs.bus_voltage_v = Parameter.fVdc;
	inputs.electrical_theta_rad = electrical_theta;
	inputs.mechanical_position_counts = Parameter.fPosition;
	inputs.mechanical_speed_rpm = Parameter.fActSpeed;
	inputs.encoder_resolution_counts = (MotorParameter[MOTOR_ENCODER_RESOLUTION] > 0.0f) ?
		MotorParameter[MOTOR_ENCODER_RESOLUTION] : (float)MOTOR_ENC_RES;
	inputs.rated_current_a = (MotorParameter[MOTOR_RATED_CURRENT_RMS] > 0.0f) ?
		MotorParameter[MOTOR_RATED_CURRENT_RMS] : DEFAULT_MOTOR_RATED_CURRENT_RMS;
	inputs.rotor_inertia = MotorParameter[MOTOR_ROTOR_INERTIA] * 0.001f;

	MotorAutoTune_Process(&gMotorAutoTune, &inputs, &outputs);

	gIdRefA = 0.0f;
	gIqRefA = 0.0f;
	gTargetSpeedRpm = 0.0f;
	gVfFrequencyHz = 0.0f;
	gVfVoltageV = 0.0f;

	switch (outputs.mode)
	{
		case MOTOR_AUTOTUNE_OUTPUT_CURRENT_LOOP:
			voltage_limit = outputs.voltage_limit_v;
			if (voltage_limit <= 0.0f)
			{
				voltage_limit = Parameter.fVdc * 0.30f;
			}
			gIdRefA = outputs.id_ref_a;
			gIqRefA = outputs.iq_ref_a;
			RunCurrentLoopForTheta(
				electrical_theta,
				outputs.id_ref_a,
				outputs.iq_ref_a,
				voltage_limit,
				outputs.isolate_q_axis);
			break;

		case MOTOR_AUTOTUNE_OUTPUT_DIRECT_D_VOLTAGE:
			ApplyVoltageVectorForTheta(electrical_theta, outputs.vd_voltage_v, outputs.vq_voltage_v);
			break;

		case MOTOR_AUTOTUNE_OUTPUT_OPEN_LOOP_VF:
			gVfFrequencyHz = outputs.vf_frequency_hz;
			gVfVoltageV = outputs.vf_voltage_v;
			pole_pairs = (Parameter.u8PolePair > 0u) ? (float)Parameter.u8PolePair : 1.0f;
			gTargetSpeedRpm = (outputs.vf_frequency_hz * 60.0f) / pole_pairs;
			ApplyOpenLoopVfCommand(outputs.vf_frequency_hz, outputs.vf_voltage_v);
			break;

		case MOTOR_AUTOTUNE_OUTPUT_DISABLED:
		default:
			Parameter.fVdq[0] = 0.0f;
			Parameter.fVdq[1] = 0.0f;
			Parameter.fVabc[0] = 0.0f;
			Parameter.fVabc[1] = 0.0f;
			Parameter.fVabc[2] = 0.0f;
			GeneratePWM(0.5f, 0.5f, 0.5f);
			break;
	}

	if ((gMotorAutoTune.state == MOTOR_AUTOTUNE_STATE_DONE) ||
		(gMotorAutoTune.state == MOTOR_AUTOTUNE_STATE_ERROR))
	{
		STM_NextState(&StateMachine, STOP);
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
   HAL_Init();

  /* USER CODE BEGIN Init */
	
	
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	InitIsrDebugCounter();

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM8_Init();
  MX_TIM1_Init();
  MX_FSMC_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	
	// Init paramter and enable after ADC init
	
	Init_Parameter(&Parameter);
	Reset_CurrentSensor(&Current_Sensor);
	STM_Init(&StateMachine);
	memset(&USB_Comm, 0, sizeof(USB_Comm));
	MotorAutoTune_Reset(&gMotorAutoTune);
	LoadDefaultParameters();
	(void)LoadParametersFromFlashIfAvailable();
	UpdateDriverParameter(DriverParameter);
	UpdateMotorParameter(MotorParameter);
	gEncoderAlignmentLastCapturedOffset = Parameter.Offset_Enc;
	ApplyControlTimingMode(USER_DEFAULT_CONTROL_TIMING_MODE);
	
	// Need this for sync 16kHz interupt from FPGA
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, HALF_PWM_PERIOD);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);

	HAL_GPIO_WritePin(oEnableReadADC_GPIO_Port, oEnableReadADC_Pin, GPIO_PIN_SET); //==> required for current reading value
	
	// Keep FPGA encoder parser aligned with the active motor parameter set ==> with this driver required for reading encoder value.
	*(__IO uint16_t*)(REG_ENCODER_ID) = (uint16_t)MotorParameter[MOTOR_ENCODER_ID];
	gFpgaEncoderParserConfigured = 0u;
	gFpgaDoneLatched = 0u;
	gFpgaDoneTickMs = HAL_GetTick();
	ServiceFpgaStartup();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	ServiceFpgaStartup();
	UpdateIsrMeasureOnlyFrequency();
	USB_ProcessData(&USB_Comm);
	USB_TransmitData(&USB_Comm);
	
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
	
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
  htim1.Init.Period = 10500;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM8 init function */
static void MX_TIM8_Init(void)
{

  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
  htim8.Init.Period = 10500;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_ETRF;
  sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_INVERTED;
  sSlaveConfig.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchronization(&htim8, &sSlaveConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 202;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim8);

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(oECATErrorLed_GPIO_Port, oECATErrorLed_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(oFaultACK_GPIO_Port, oFaultACK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, oEnableReadADC_Pin|oResetEncoder_Pin|oOutput2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, oOutput4_Pin|oOutput3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(oOutput1_GPIO_Port, oOutput1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, oDriveErrorLed_Pin|oDriveRunLed_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(oTLED3_GPIO_Port, oTLED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, oTLED2_Pin|oTLED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : oECATErrorLed_Pin oTLED2_Pin oTLED1_Pin */
  GPIO_InitStruct.Pin = oECATErrorLed_Pin|oTLED2_Pin|oTLED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : ECAT_IRQ_Pin ECAT_SYNC0_Pin ECAT_SYNC1_Pin */
  GPIO_InitStruct.Pin = ECAT_IRQ_Pin|ECAT_SYNC0_Pin|ECAT_SYNC1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : i15V_State_Pin iIPM_Fault_Pin iFanConnect_Pin iFPGA_DONE_Pin
                           iUSB_VBUS_Pin */
  GPIO_InitStruct.Pin = i15V_State_Pin|iIPM_Fault_Pin|iFanConnect_Pin|iFPGA_DONE_Pin
                          |iUSB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : iEncFaultIsr_Pin */
  GPIO_InitStruct.Pin = iEncFaultIsr_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(iEncFaultIsr_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : iMeasureIsr_Pin */
  GPIO_InitStruct.Pin = iMeasureIsr_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(iMeasureIsr_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : oFaultACK_Pin */
  GPIO_InitStruct.Pin = oFaultACK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(oFaultACK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : oEnableReadADC_Pin oResetEncoder_Pin oOutput2_Pin oOutput1_Pin */
  GPIO_InitStruct.Pin = oEnableReadADC_Pin|oResetEncoder_Pin|oOutput2_Pin|oOutput1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : iBrakeResConnect_Pin iInput1_Pin iUSB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = iBrakeResConnect_Pin|iInput1_Pin|iUSB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : oOutput4_Pin oOutput3_Pin */
  GPIO_InitStruct.Pin = oOutput4_Pin|oOutput3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : iInput6_Pin */
  GPIO_InitStruct.Pin = iInput6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(iInput6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : iInput5_Pin iInput4_Pin iInput3_Pin iInput2_Pin */
  GPIO_InitStruct.Pin = iInput5_Pin|iInput4_Pin|iInput3_Pin|iInput2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : oDriveErrorLed_Pin oDriveRunLed_Pin oTLED3_Pin */
  GPIO_InitStruct.Pin = oDriveErrorLed_Pin|oDriveRunLed_Pin|oTLED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{
  FSMC_NORSRAM_TimingTypeDef Timing;

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  hsram1.Init.PageSize = FSMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 1;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 70;
  Timing.BusTurnAroundDuration = 6;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /** Perform the SRAM2 memory initialization sequence
  */
  hsram2.Instance = FSMC_NORSRAM_DEVICE;
  hsram2.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram2.Init */
  hsram2.Init.NSBank = FSMC_NORSRAM_BANK2;
  hsram2.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram2.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram2.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram2.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram2.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram2.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram2.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram2.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram2.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram2.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram2.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram2.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  hsram2.Init.PageSize = FSMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 1;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 6;
  Timing.BusTurnAroundDuration = 1;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram2, &Timing, NULL) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USER CODE BEGIN 4 */

static void ToggleIsrScopeProbe(void)
{
	HAL_GPIO_TogglePin(oTLED1_GPIO_Port, oTLED1_Pin);
}

// Loop EXT_Interupt from FPGA
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint16_t fault = NO_ERROR;
	uint16_t calib_fault = NO_ERROR;

	if (GPIO_Pin != iMeasureIsr_Pin)
	{
		return;
	}

	ToggleIsrScopeProbe();
	UpdateIsrDebugPeriod();
	gIsrMeasureEdgeCount++;
	if (gIsrMeasureOnlyMode != 0u)
	{
		return;
	}

	ReadFastProtectionFeedback();
	// OR two ERRORS command
	fault = CheckVbusFault(&Parameter);
	fault |= CheckTempFault(&Parameter);

	if ((Current_Sensor.CalibFinish > 0) && ((StateMachine.bState == RUN) || (StateMachine.bState == ENCODER_ALIGN)))
	{
		// OR Current fault when running (Just in state RUNNING) after calib prior to this state
		fault |= CheckCurrentPhaseFault(&Current_Sensor, Parameter.fIabc[0], Parameter.fIabc[1], Parameter.fIabc[2]);
	}

	if (fault != NO_ERROR)
	{
		ReportFault(fault);
		return;
	}

	GetParameter(&Parameter, MotorParameter);
	
	// Calculate speed and Parameter.ftheta for FOC control
	UpdateMeasuredSpeedAndTheta();

	if ((StateMachine.bState == FAULT_NOW) || (StateMachine.bState == FAULT_OVER))
	{
		SetPwmEnabled(0u);
		system_on = 0u;
		return;
	}

	if (StateMachine.bState == OFFSET_CALIB)
	{
		if (Current_Sensor.CalibCounter == 0)
		{
			Parameter.u16Offset_Ia = OFFSET;
			Parameter.u16Offset_Ib = OFFSET;
		}

		SetPwmEnabled(1u);
		GeneratePWM(0.5f, 0.5f, 0.5f);
		calib_fault = CalibrateCurrentSensor(&Current_Sensor, &Parameter);
		if (calib_fault != NO_ERROR)
		{
			ReportFault(calib_fault);
			return;
		}

		if (Current_Sensor.CalibFinish > 0)
		{
			ResetControlLoops();
			if (gEncoderAlignmentRequested != 0u)
			{
				STM_NextState(&StateMachine, ENCODER_ALIGN);
			}
			else if (gServoArmOnlyRequested != 0u)
			{
				gServoArmOnlyRequested = 0u;
				STM_NextState(&StateMachine, STOP);
			}
			else
			{
				STM_NextState(&StateMachine, START);
			}
		}

		system_on = 0u;
		return;
	}

	if (StateMachine.bState == ENCODER_ALIGN)
	{
		gEncoderAlignmentStatus = ENCODER_ALIGNMENT_STATUS_RUNNING;
		SetPwmEnabled(1u);
		RunFocLoop();
		UpdateTraceCapture();
		if (IdSquareTuning.AlignmentDone != 0u)
		{
			FinalizeEncoderAlignment(1u);
		}
		system_on = 1u;
		return;
	}

	if ((StateMachine.bState == IDLE) || (StateMachine.bState == STOP))
	{
		RestoreIdSquareTuningOffsetGuard();
		IdSquareTuning.Enable = 0u;
		IdSquareTuning.AlignmentDone = 0u;
		IdSquareTuning.AlignmentCounter = 0u;
		IdSquareTuning.fPhase = 0.0f;
		IdSquareTuning.fVoltageLimitApplied = 0.0f;
		IdSquareTuning.fAlignmentCurrentApplied = 0.0f;
		IdSquareTuning.OffsetCaptured = 0;
		if ((gEncoderAlignmentStatus == ENCODER_ALIGNMENT_STATUS_REQUESTED) ||
			(gEncoderAlignmentStatus == ENCODER_ALIGNMENT_STATUS_RUNNING))
		{
			gEncoderAlignmentStatus = ENCODER_ALIGNMENT_STATUS_IDLE;
		}
		gEncoderAlignmentRequested = 0u;
		gEncoderAlignmentContinueToRun = 0u;
		gEncoderAlignmentResumeRunMode = RUN_MODE_FOC;
		gServoArmOnlyRequested = 0u;
		ResetControlLoops();
		UpdateMeasuredCurrentsForTheta(Parameter.fTheta, Parameter.fIabc[0], Parameter.fIabc[1]);
		SetPwmEnabled(0u);
		if (StateMachine.bState == STOP)
		{
			STM_NextState(&StateMachine, IDLE);
		}
		system_on = 0u;
		return;
	}

	if (StateMachine.bState == START)
	{
		ResetControlLoops();
		SetPwmEnabled(1u);
		STM_NextState(&StateMachine, RUN);
	}

	SetPwmEnabled(1u);
	if (gRunMode == RUN_MODE_OPEN_LOOP_VF)
	{
		RunOpenLoopVf();
	}
	else if (gRunMode == RUN_MODE_AUTOTUNE)
	{
		RunMotorAutoTuneLoop();
	}
	else
	{
		RunFocLoop();
	}
	UpdateTraceCapture();
	system_on = 1u;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
