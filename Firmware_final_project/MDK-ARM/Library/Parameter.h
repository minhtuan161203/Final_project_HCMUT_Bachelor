#ifndef PARAMETER_H
#define PARAMETER_H
#include "stdint.h"
//#include "VelocityControl.h"
//#include "CurrentControl.h"
//#include "PositionControl.h"
typedef struct
{
	float fVdc;
	float fVabc[3];
	float fIdq[2];
	float fTemparature;
	float fTheta;
	float fIabc[3];
	float fVdq[2];
	float fActSpeed;
	float fActSpeedFilter;
	float fPrePosition;
	float fPosition;
	uint32_t EncRes;
	int32_t EncSingleTurn;
	int32_t Pulse;
	int32_t PreEncSingleTurn;
	int32_t Offset_Enc;
	uint16_t Ui16nEncRxWordBuffer[7]; 
	uint16_t Ui16ControlFrequency;
	uint16_t u16Offset_Ia;
	uint16_t u16Offset_Ib;
	uint8_t u8IsPulseOrRPM; // = 0: Pulse, =1: PWM 
	uint8_t Ui8ServoOn;
	uint8_t u8PolePair;
	uint16_t CurrMultiturns;
	uint8_t EncoderAlarm;
	uint8_t HallSensor;
	uint8_t bInitPos;
	int32_t PrevDeltaPos;
	int32_t DeltaPos;
}Parameterhandle_t;


typedef struct
{
	uint16_t Conversionfactor;						/*Convert ADC value to current value(Ampere)*/
	float OverCurrentThreshold;						/*Threshold over current occur*/
	uint32_t Sum1, Sum2;
	int8_t CalibFinish;
	uint16_t CalibCounter;
	uint32_t CalibTimeoutCounter;
}	CurrentSensor_t;

void Reset_CurrentSensor(CurrentSensor_t * pHandle);
void GetParameter(Parameterhandle_t *pHandle, float *MotorParam);
void Init_Parameter(Parameterhandle_t *pHandle);
uint16_t CheckCurrentPhaseFault(CurrentSensor_t *pHandle, float CurrentPhaseU, float CurrentPhaseV, float CurrentPhaseW);
uint16_t CheckVbusFault(Parameterhandle_t *pHandle);
uint16_t CheckTempFault(Parameterhandle_t *pHandle);
uint16_t CalibrateCurrentSensor(CurrentSensor_t *pHandle, Parameterhandle_t *param);
//void UpdateDriverAndMotorParameter(uint32_t *DriverParameter, uint32_t *MotorParameter, CurrentControl_handle_t *pCurrent, VelocityControl_handle_t *pVelocity, PositionControl_handle_t *pPosition);
#endif
