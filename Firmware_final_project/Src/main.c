
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
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim8;

SRAM_HandleTypeDef hsram1;
SRAM_HandleTypeDef hsram2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

float DriverParameter[16];
float MotorParameter[32];
uint16_t FaultCode = NO_ERROR;

uint8_t system_on = 0;
volatile float gTargetSpeedRpm = 0.0f;
volatile uint8_t gRunMode = RUN_MODE_FOC;
volatile float gVfFrequencyHz = 0.0f;
volatile float gVfVoltageV = 0.0f;
volatile float gOpenLoopFrequencyLimitHz = DEFAULT_MOTOR_RATED_ELECTRICAL_FREQUENCY_HZ;
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
float RecordTable1[TRACE_DATA_LENGTH * 4u];
TraceData Trace_Data;
IdSquareTuning_t IdSquareTuning;

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
static void RunOpenLoopVf(void);
static void ReportFault(uint16_t fault);
static void RunFocLoop(void);
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
static uint8_t IsAbsoluteEncoderId(uint32_t encoder_id);
static void RefreshEncoderAlignmentPolicy(void);
static void FinalizeEncoderAlignment(uint8_t alignment_successful);
static uint8_t LoadParametersFromFlashIfAvailable(void);
void ApplyControlTimingMode(uint8_t mode);
void PrepareEncoderAlignment(uint8_t continue_to_run, uint8_t resume_run_mode);
uint8_t SaveParametersToFlash(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define ID_SQUARE_TUNING_MAX_RATED_CURRENT_RATIO 0.20f
#define ID_SQUARE_TUNING_MAX_OC_RATIO 0.20f
#define ID_SQUARE_TUNING_MAX_VOLTAGE_RATIO 0.20f
#define ID_SQUARE_TUNING_MAX_FREQUENCY_HZ 50.0f
#define ID_SQUARE_TUNING_MIN_VOLTAGE_LIMIT_V 1.0f
#define ID_SQUARE_TUNING_ALIGN_CURRENT_MIN_A 0.2f
#define ID_SQUARE_TUNING_ALIGN_TIME_S 0.25f
#define ID_SQUARE_ANGLE_TEST_NONE 0u
#define ID_SQUARE_ANGLE_TEST_PLUS_90 1u
#define ID_SQUARE_ANGLE_TEST_MINUS_90 2u
#define ID_SQUARE_ANGLE_TEST_PLUS_180 3u
#define SPEED_ESTIMATE_LPF_ALPHA 0.1f
#define DEBUG_AVG_SAMPLES 256u
#define DRIVER_PARAMETER_COUNT 16u
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
	IdSquareTuning.AlignmentCounter = 0u;
	IdSquareTuning.fPhase = 0.0f;
	IdSquareTuning.fVoltageLimitApplied = 0.0f;
	IdSquareTuning.fAlignmentCurrentApplied = 0.0f;
	IdSquareTuning.OffsetCaptured = 0;
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

	if (mode == CONTROL_TIMING_MODE_3KHZ)
	{
		current_loop_hz = USER_ISR_FREQUENCY_3KHZ;
	}
	else
	{
		mode = CONTROL_TIMING_MODE_16KHZ;
		current_loop_hz = USER_ISR_FREQUENCY_16KHZ;
	}

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
	gIdRefA = 0.0f;
	gIqRefA = 0.0f;
	gSpeedLoopDivider = 0u;
	gDebugOpenLoopElectricalHzCmd = 0.0f;
	gDebugOpenLoopSyncRpmCmd = 0.0f;
	gDebugExpectedDeltaPosSync = 0.0f;
	ResetDebugAveraging();
	ResetControl_V_over_F();
	GeneratePWM(0.5f, 0.5f, 0.5f);
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
	float voltage_limit = 0.5f * Parameter.fVdc * ID_SQUARE_TUNING_MAX_VOLTAGE_RATIO;
	float motor_voltage_limit = MotorParameter[MOTOR_MAXIMUM_VOLTAGE] * ID_SQUARE_TUNING_MAX_VOLTAGE_RATIO;

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

	return (IdSquareTuning.fPhase < 0.5f) ? IdSquareTuning.fAmplitudeApplied : -IdSquareTuning.fAmplitudeApplied;
}

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

static int32_t CalcAlignedEncoderOffset(float encoder_resolution)
{
	float pole_pairs;
	float electrical_offset_counts;
	float single_turn_position;
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
	float speed_limit;

	if (driver_parameter == 0)
	{
		return;
	}

	if (driver_parameter[SPEED_P_GAIN] <= 0.0f)
	{
		driver_parameter[SPEED_P_GAIN] = 0.02f;
	}
	if (driver_parameter[SPEED_I_GAIN] <= 0.0f)
	{
		driver_parameter[SPEED_I_GAIN] = 5.0f;
	}
	if (driver_parameter[MAXIMUM_SPEED] <= 0.0f)
	{
		driver_parameter[MAXIMUM_SPEED] = 1500.0f;
	}

	max_speed = driver_parameter[MAXIMUM_SPEED];
	speed_limit = ((MotorParameter[MOTOR_RATED_CURRENT_RMS] > 0.0f) ?
		MotorParameter[MOTOR_RATED_CURRENT_RMS] : DIRVER_OVER_CURRENT_THRESHOLD_AMPERE_UNIT);

	gSpeedPi.fDtSec = 1.0f / GetEffectiveSpeedLoopFrequency();
	gSpeedPi.fKp = driver_parameter[SPEED_P_GAIN];
	gSpeedPi.fKi = driver_parameter[SPEED_I_GAIN];
	gSpeedPi.fUpOutLim = speed_limit;
	gSpeedPi.fLowOutLim = -speed_limit;

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
	RefreshEncoderAlignmentPolicy();
	UpdateDriverParameter(DriverParameter);
}

static void LoadDefaultParameters(void)
{
	memset(DriverParameter, 0, sizeof(DriverParameter));
	memset(MotorParameter, 0, sizeof(MotorParameter));

	DriverParameter[CONTROL_MODE] = (float)SPEED_CONTROL_MODE;
	DriverParameter[SPEED_P_GAIN] = 0.02f;
	DriverParameter[SPEED_I_GAIN] = 5.0f;
	DriverParameter[MAXIMUM_SPEED] = DEFAULT_MOTOR_RATED_SPEED_RPM;
	DriverParameter[SPEED_UNIT] = 0.0f;

	MotorParameter[MOTOR_RATED_CURRENT_RMS] = DEFAULT_MOTOR_RATED_CURRENT_RMS;
	MotorParameter[MOTOR_PEAK_CURRENT_RMS] = DEFAULT_MOTOR_PEAK_CURRENT_RMS;
	MotorParameter[MOTOR_ENCODER_ID] = (float)DEFAULT_MOTOR_ENCODER_ID;
	MotorParameter[MOTOR_ENCODER_RESOLUTION] = (float)MOTOR_ENC_RES;
	MotorParameter[MOTOR_MAXIMUM_POWER] = DEFAULT_MOTOR_MAXIMUM_POWER_W;
	MotorParameter[MOTOR_MAXIMUM_VOLTAGE] = DEFAULT_MOTOR_MAXIMUM_VOLTAGE_V;
	MotorParameter[MOTOR_MAXIMUM_SPEED] = DEFAULT_MOTOR_RATED_SPEED_RPM;
	MotorParameter[MOTOR_NUMBER_POLE_PAIRS] = (float)INITIAL_MOTOR_POLE_PAIRS;
	MotorParameter[MOTOR_CURRENT_P_GAIN] = 1.0f;
	MotorParameter[MOTOR_CURRENT_I_GAIN] = 150.0f;
	MotorParameter[MOTOR_CURRENT_CTRL_DIRECTION] = 1.0f;
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

	mechanical_angle = (position_single_turn / encoder_resolution) * (2.0f * PI);
	electrical_angle = mechanical_angle * (float)Parameter.u8PolePair;
	gDebugMechanicalAngleRad = WrapAngle(mechanical_angle);
	gDebugElectricalAngleRad = WrapAngle(electrical_angle);
	Parameter.fTheta = WrapAngle(electrical_angle);
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

static void RunOpenLoopVf(void)
{
	float frequency_limit_hz = GetOpenLoopFrequencyLimitHz();
	float requested_frequency;
	float requested_voltage = ClampFloat(gVfVoltageV, 0.0f, GetOpenLoopVoltageLimit());
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
	requested_frequency = ClampFloat(gVfFrequencyHz, 0.0f, frequency_limit_hz);

	if (Parameter.fVdc > 1.0f)
	{
		modulation = requested_voltage / Parameter.fVdc;
	}
	modulation = ClampFloat(modulation, 0.0f, 0.85f);

	gIdRefA = 0.0f;
	gIqRefA = 0.0f;
	gTargetSpeedRpm = (requested_frequency * 60.0f) / pole_pairs;
	gDebugOpenLoopElectricalHzCmd = requested_frequency;
	gDebugOpenLoopSyncRpmCmd = gTargetSpeedRpm;
	gDebugExpectedDeltaPosSync = (requested_frequency * (MotorParameter[MOTOR_ENCODER_RESOLUTION] > 0.0f ?
		MotorParameter[MOTOR_ENCODER_RESOLUTION] : (float)MOTOR_ENC_RES)) /
		(GetEffectiveCurrentLoopFrequency() * pole_pairs);
	vf_output = Control_V_over_F(requested_frequency, modulation);

	Parameter.fVabc[0] = (vf_output.U - 0.5f) * Parameter.fVdc;
	Parameter.fVabc[1] = (vf_output.V - 0.5f) * Parameter.fVdc;
	Parameter.fVabc[2] = (vf_output.W - 0.5f) * Parameter.fVdc;

	sin_theta = sinf(Parameter.fTheta);
	cos_theta = cosf(Parameter.fTheta);

	gClarke.fA = Parameter.fIabc[0];
	gClarke.fB = Parameter.fIabc[1];
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

static void ReportFault(uint16_t fault)
{
	FaultCode |= fault;
	gDebugFaultSnapshot = FaultCode;
	USB_Comm.SendError = true;
	gRunMode = RUN_MODE_FOC;
	gVfFrequencyHz = 0.0f;
	gVfVoltageV = 0.0f;
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
	float current_temp;
	float encoder_resolution;
	uint8_t alignment_active = 0u;
	uint8_t alignment_hold_mode = 0u;

	if (Parameter.fVdc < 1.0f)
	{
		GeneratePWM(0.5f, 0.5f, 0.5f);
		return;
	}

	encoder_resolution = (MotorParameter[MOTOR_ENCODER_RESOLUTION] > 0.0f) ?
		MotorParameter[MOTOR_ENCODER_RESOLUTION] : (float)MOTOR_ENC_RES;
	alignment_hold_mode = ((IdSquareTuning.Enable != 0u) &&
		(IdSquareTuning.Mode == ID_SQUARE_TUNING_MODE_ALIGNMENT_HOLD) &&
		(IdSquareTuning.AlignmentDone != 0u)) ? 1u : 0u;
	control_theta = Parameter.fTheta;
	if ((IdSquareTuning.Enable != 0u) && (IdSquareTuning.AlignmentDone == 0u))
	{
		control_theta = 0.0f;
		alignment_active = 1u;
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

	sin_theta = sinf(control_theta);
	cos_theta = cosf(control_theta);

	current_phase_u = Parameter.fIabc[0];
	current_phase_v = Parameter.fIabc[1];
	if (IdSquareTuning.Enable != 0u)
	{
		if (IdSquareTuning.CurrentUvSwapTest != 0u)
		{
			current_temp = current_phase_u;
			current_phase_u = current_phase_v;
			current_phase_v = current_temp;
		}
		if (IdSquareTuning.CurrentPolarityInvertTest != 0u)
		{
			current_phase_u = -current_phase_u;
			current_phase_v = -current_phase_v;
		}
	}
	
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
			gSpeedLoopDivider = 0u;
			gSpeedPi.fIn = gTargetSpeedRpm - Parameter.fActSpeed;
			gSpeedPi.m_calc(&gSpeedPi);
			gIqRefA = ClampFloat(gSpeedPi.fOut, gSpeedPi.fLowOutLim, gSpeedPi.fUpOutLim);
		}

		voltage_limit = Parameter.fVdc * 0.45f;
	}

	gIdPi.fUpOutLim = voltage_limit;
	gIdPi.fLowOutLim = -voltage_limit;
	gIqPi.fUpOutLim = voltage_limit;
	gIqPi.fLowOutLim = -voltage_limit;

	gIdPi.fIn = gIdRefA - gPark.fD;
	gIdPi.m_calc(&gIdPi);
	
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
	else
	{
		gIqPi.fIn = gIqRefA - gPark.fQ;
		gIqPi.m_calc(&gIqPi);
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
		else
		{
			IdSquareTuning.OffsetCaptured = CalcAlignedEncoderOffset(encoder_resolution);
			Parameter.Offset_Enc = IdSquareTuning.OffsetCaptured;
			MotorParameter[MOTOR_HALL_OFFSET] = (float)IdSquareTuning.OffsetCaptured;
			IdSquareTuning.AlignmentDone = 1u;
			IdSquareTuning.AlignmentCounter = 0u;
			ResetControlLoops();
		}
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
	LoadDefaultParameters();
	(void)LoadParametersFromFlashIfAvailable();
	UpdateDriverParameter(DriverParameter);
	UpdateMotorParameter(MotorParameter);
	gEncoderAlignmentLastCapturedOffset = Parameter.Offset_Enc;
	ApplyControlTimingMode(USER_DEFAULT_CONTROL_TIMING_MODE);
	HAL_GPIO_WritePin(oEnableReadADC_GPIO_Port, oEnableReadADC_Pin, GPIO_PIN_SET); //==> required for current reading value
	
	// Keep FPGA encoder parser aligned with the active motor parameter set ==> with this driver required for reading encoder value.
	*(__IO uint16_t*)(REG_ENCODER_ID) = (uint16_t)MotorParameter[MOTOR_ENCODER_ID];

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

// Loop EXT_Interupt from FPGA
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint16_t fault = NO_ERROR;
	uint16_t calib_fault = NO_ERROR;

	if (GPIO_Pin != iMeasureIsr_Pin)
	{
		return;
	}

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
		ResetControlLoops();
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




