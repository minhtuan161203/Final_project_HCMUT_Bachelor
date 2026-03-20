#include "Parameter.h"
#include "math.h"
#include "stm32f4xx_hal.h"
#include "string.h"
#include "define.h"


extern uint16_t FaultCode;
extern CurrentSensor_t Current_Sensor;

void GetParameter(Parameterhandle_t *pHandle, float *MotorParam)
{
//		static int32_t PrevDeltaPos = 0,DeltaPos = 0;
	
		pHandle->Ui16nEncRxWordBuffer[0] = *(__IO uint16_t*)(REG_ENCODER_RX_WORD0);
		pHandle->Ui16nEncRxWordBuffer[1] = *(__IO uint16_t*)(REG_ENCODER_RX_WORD1);
		pHandle->Ui16nEncRxWordBuffer[2] = *(__IO uint16_t*)(REG_ENCODER_RX_WORD2);
		pHandle->Ui16nEncRxWordBuffer[3] = *(__IO uint16_t*)(REG_ENCODER_RX_WORD3);
		pHandle->Ui16nEncRxWordBuffer[4] = *(__IO uint16_t*)(REG_ENCODER_RX_WORD4);
		pHandle->Ui16nEncRxWordBuffer[5] = *(__IO uint16_t*)(REG_ENCODER_RX_WORD5);
		pHandle->Ui16nEncRxWordBuffer[6] = *(__IO uint16_t*)(REG_ENCODER_RX_WORD6);
	
		switch((uint32_t)MotorParam[MOTOR_ENCODER_ID])
		{
			case TAMAGAWA_SERIAL_ABS_SINGLE_TURN:
			{
				pHandle->EncSingleTurn = (int32_t)(((uint32_t)(pHandle->Ui16nEncRxWordBuffer[2] & 0xFF) << 16) | (uint32_t)(pHandle->Ui16nEncRxWordBuffer[1]));
				pHandle->PrevDeltaPos = pHandle->DeltaPos;
				if(pHandle->bInitPos == 0)
				{
					pHandle->bInitPos = 1;
					pHandle->DeltaPos = 0;
				}
				else
					if(MotorParam[MOTOR_CURRENT_CTRL_DIRECTION] == 0)
						pHandle->DeltaPos = pHandle->EncSingleTurn - pHandle->PreEncSingleTurn;
					else
						pHandle->DeltaPos = -pHandle->EncSingleTurn + pHandle->PreEncSingleTurn;
				if(pHandle->DeltaPos > ((int32_t)MotorParam[MOTOR_ENCODER_RESOLUTION] >> 1)) 					pHandle->DeltaPos -= (int32_t)MotorParam[MOTOR_ENCODER_RESOLUTION];
				else if(pHandle->DeltaPos < -((int32_t)MotorParam[MOTOR_ENCODER_RESOLUTION] >> 1)) 		pHandle->DeltaPos += (int32_t)MotorParam[MOTOR_ENCODER_RESOLUTION];
				pHandle->PreEncSingleTurn = pHandle->EncSingleTurn;
				pHandle->fPosition += pHandle->DeltaPos;
				pHandle->HallSensor = 0;
				pHandle->CurrMultiturns = 0;
				pHandle->EncoderAlarm = 0;
			}
			break;
			case TAMAGAWA_SERIAL_ABS_MULTI_TURN:
			case TAMAGAWA_SERIAL_ABS_MULTI_TURN_23BIT:
			{
				//Get single turn
				pHandle->EncSingleTurn = (int32_t)(((uint32_t)(pHandle->Ui16nEncRxWordBuffer[2] & 0xFF) << 16) | (uint32_t)(pHandle->Ui16nEncRxWordBuffer[1]));
				//Get multi turn if use absolute encoder
				pHandle->CurrMultiturns = pHandle->Ui16nEncRxWordBuffer[3];			
				pHandle->PrevDeltaPos = pHandle->DeltaPos;	
				if(pHandle->bInitPos == 0)
				{
					pHandle->bInitPos = 1;
					pHandle->DeltaPos = 0;
				}
				else
					if(MotorParam[MOTOR_CURRENT_CTRL_DIRECTION] == 0)
						pHandle->DeltaPos = pHandle->EncSingleTurn - pHandle->PreEncSingleTurn;
					else
						pHandle->DeltaPos = -pHandle->EncSingleTurn + pHandle->PreEncSingleTurn;
				if(pHandle->DeltaPos > ((int32_t)MotorParam[MOTOR_ENCODER_RESOLUTION]>>1)) 					pHandle->DeltaPos -= (int32_t)MotorParam[MOTOR_ENCODER_RESOLUTION];
				else if(pHandle->DeltaPos < -((int32_t)MotorParam[MOTOR_ENCODER_RESOLUTION]>>1)) 		pHandle->DeltaPos += (int32_t)MotorParam[MOTOR_ENCODER_RESOLUTION];
				pHandle->PreEncSingleTurn = pHandle->EncSingleTurn;
				if((int32_t)MotorParam[MOTOR_ABS_ENCODER_MODE] == 0)
				{
					if(MotorParam[MOTOR_CURRENT_CTRL_DIRECTION] == 0)
						pHandle->fPosition = pHandle->CurrMultiturns * MotorParam[MOTOR_ENCODER_RESOLUTION] + pHandle->EncSingleTurn;
					else
						pHandle->fPosition = -pHandle->CurrMultiturns * MotorParam[MOTOR_ENCODER_RESOLUTION] - pHandle->EncSingleTurn;
				}
				else
					pHandle->fPosition += pHandle->DeltaPos;
				pHandle->EncoderAlarm = pHandle->Ui16nEncRxWordBuffer[4] >> 8;
				pHandle->HallSensor = 0;
			}
			break;			
			case PANASONIC_MINAS_A5_SERIAL_INC:
			{
				pHandle->HallSensor = (pHandle->Ui16nEncRxWordBuffer[0] >> 8) & 0x0F;
				pHandle->EncSingleTurn = (uint32_t)((pHandle->Ui16nEncRxWordBuffer[3] & 0x0FFF) << 8) | (uint32_t)(pHandle->Ui16nEncRxWordBuffer[2] >> 8);
				pHandle->PrevDeltaPos = pHandle->DeltaPos;
				if(pHandle->bInitPos == 0)
				{
					pHandle->bInitPos = 1;
					pHandle->DeltaPos = 0;
				}
				else
					if(MotorParam[MOTOR_CURRENT_CTRL_DIRECTION] == 0)
						pHandle->DeltaPos = pHandle->EncSingleTurn - pHandle->PreEncSingleTurn;
					else
						pHandle->DeltaPos = -pHandle->EncSingleTurn + pHandle->PreEncSingleTurn;
				if(pHandle->DeltaPos > ((int32_t)MotorParam[MOTOR_ENCODER_RESOLUTION]>>1)) 					pHandle->DeltaPos -= (int32_t)MotorParam[MOTOR_ENCODER_RESOLUTION];
				else if(pHandle->DeltaPos < -((int32_t)MotorParam[MOTOR_ENCODER_RESOLUTION]>>1)) 		pHandle->DeltaPos += (int32_t)MotorParam[MOTOR_ENCODER_RESOLUTION];
				pHandle->PreEncSingleTurn = pHandle->EncSingleTurn;
				pHandle->fPosition += pHandle->DeltaPos;
				pHandle->CurrMultiturns = 0;
				pHandle->EncoderAlarm = 0;
			}			
			break;
			case PANASONIC_MINAS_A5_SERIAL_ABS:
			{
				//Get single turn
				pHandle->EncSingleTurn = (uint32_t)((pHandle->Ui16nEncRxWordBuffer[2] & 0x01) << 16) | (uint32_t)(pHandle->Ui16nEncRxWordBuffer[1]);
				//Get multi turn if use absolute encoder
				pHandle->CurrMultiturns = ((pHandle->Ui16nEncRxWordBuffer[3] & 0xFF) << 8) | (pHandle->Ui16nEncRxWordBuffer[2] >> 8);			
				pHandle->PrevDeltaPos = pHandle->DeltaPos;	
				if(pHandle->bInitPos == 0)
				{
					pHandle->bInitPos = 1;
					pHandle->DeltaPos = 0;
				}
				else
					if(MotorParam[MOTOR_CURRENT_CTRL_DIRECTION] == 0)
						pHandle->DeltaPos = pHandle->EncSingleTurn - pHandle->PreEncSingleTurn;
					else
						pHandle->DeltaPos = -pHandle->EncSingleTurn + pHandle->PreEncSingleTurn;
				if(pHandle->DeltaPos > ((int32_t)MotorParam[MOTOR_ENCODER_RESOLUTION]>>1)) 					pHandle->DeltaPos -= (int32_t)MotorParam[MOTOR_ENCODER_RESOLUTION];
				else if(pHandle->DeltaPos < -((int32_t)MotorParam[MOTOR_ENCODER_RESOLUTION]>>1)) 		pHandle->DeltaPos += (int32_t)MotorParam[MOTOR_ENCODER_RESOLUTION];
				pHandle->PreEncSingleTurn = pHandle->EncSingleTurn;
				if(MotorParam[MOTOR_ABS_ENCODER_MODE] == 0)
				{
					if(MotorParam[MOTOR_CURRENT_CTRL_DIRECTION] == 0)
						pHandle->fPosition = pHandle->CurrMultiturns * MotorParam[MOTOR_ENCODER_RESOLUTION] + pHandle->EncSingleTurn;
					else
						pHandle->fPosition = -pHandle->CurrMultiturns * MotorParam[MOTOR_ENCODER_RESOLUTION] - pHandle->EncSingleTurn;
				}
				else
					pHandle->fPosition += pHandle->DeltaPos;
				pHandle->EncoderAlarm = pHandle->Ui16nEncRxWordBuffer[0] >> 8;
				pHandle->HallSensor = 0;
			}
			break;
			
			case PANASONIC_MINAS_A6_SERIAL_ABS:
			{
				pHandle->EncSingleTurn = (int32_t)(((uint32_t)(pHandle->Ui16nEncRxWordBuffer[2] & 0x007F) << 16) | (uint32_t)pHandle->Ui16nEncRxWordBuffer[1]);
				//Get multi turn if use absolute encoder 
				pHandle->CurrMultiturns = ((pHandle->Ui16nEncRxWordBuffer[3] & 0xFF) << 8) | (pHandle->Ui16nEncRxWordBuffer[2] >> 8);
				pHandle->PrevDeltaPos = pHandle->DeltaPos;	
				if(pHandle->bInitPos == 0)
				{
					pHandle->bInitPos = 1;
					pHandle->DeltaPos = 0;
				}
				else
					if(MotorParam[MOTOR_CURRENT_CTRL_DIRECTION] == 0)
						pHandle->DeltaPos = pHandle->EncSingleTurn - pHandle->PreEncSingleTurn;
					else
						pHandle->DeltaPos = -pHandle->EncSingleTurn + pHandle->PreEncSingleTurn;
				if(pHandle->DeltaPos > ((int32_t)MotorParam[MOTOR_ENCODER_RESOLUTION]>>1)) 					pHandle->DeltaPos -= (int32_t)MotorParam[MOTOR_ENCODER_RESOLUTION];
				else if(pHandle->DeltaPos < -((int32_t)MotorParam[MOTOR_ENCODER_RESOLUTION]>>1)) 		pHandle->DeltaPos += (int32_t)MotorParam[MOTOR_ENCODER_RESOLUTION];
					pHandle->PreEncSingleTurn = pHandle->EncSingleTurn;
				if((int32_t)MotorParam[MOTOR_ABS_ENCODER_MODE] == 0)
				{
					if(MotorParam[MOTOR_CURRENT_CTRL_DIRECTION] == 0)
						pHandle->fPosition = pHandle->CurrMultiturns * MotorParam[MOTOR_ENCODER_RESOLUTION] + pHandle->EncSingleTurn;
					else
						pHandle->fPosition = -pHandle->CurrMultiturns * MotorParam[MOTOR_ENCODER_RESOLUTION] - pHandle->EncSingleTurn;
				}
				else
					pHandle->fPosition += pHandle->DeltaPos;
				pHandle->EncoderAlarm = pHandle->Ui16nEncRxWordBuffer[3] >> 8;
				pHandle->HallSensor = 0;
			}
			break;
			default:
				break;
		}
	
// Mr.Thinh comment		
//		pHandle->EncSingleTurn = (int32_t)(((uint32_t)(pHandle->Ui16nEncRxWordBuffer[1] & 0xFF) << 16) | (uint32_t)(pHandle->Ui16nEncRxWordBuffer[0]));
//		if((pHandle->EncSingleTurn - pHandle->PreEncSingleTurn)>0xffff)
//		{
//			pHandle->Pulse += pHandle->EncSingleTurn -  pHandle->PreEncSingleTurn - pHandle->EncRes;	
//		}
//		else if((pHandle->EncSingleTurn - pHandle->PreEncSingleTurn)<-0xffff)
//		{
//			pHandle->Pulse += pHandle->EncSingleTurn -  pHandle->PreEncSingleTurn + pHandle->EncRes;
//		}
//		else 
//		{
//			pHandle->Pulse += pHandle->EncSingleTurn -  pHandle->PreEncSingleTurn;		 
//		}
//		pHandle->PreEncSingleTurn = pHandle->EncSingleTurn;
		
		pHandle->fVdc = (float)(*(__IO uint16_t*)(REG_DC_BUS_VOLTAGE)-OFFSET)/Resolution16bits*INPUT_RANGE_VDC;
		pHandle->fIabc[0] = -(float)((int32_t)*(__IO uint16_t*)(REG_CURRENT_PHASE_U)-(int32_t)pHandle->u16Offset_Ia)/Resolution16bits*INPUT_RANGE_I;
		pHandle->fIabc[1] = -(float)((int32_t)*(__IO uint16_t*)(REG_CURRENT_PHASE_V)-(int32_t)pHandle->u16Offset_Ib)/Resolution16bits*INPUT_RANGE_I;
		pHandle->fIabc[2] = - pHandle->fIabc[0] - pHandle->fIabc[1];
		pHandle->fTemparature = (float)(*(__IO uint16_t*)(REG_TEMPARATURE_SENSOR)-OFFSET)/Resolution16bits*INPUT_RANGE_TEMPARATURE;
}


void Init_Parameter(Parameterhandle_t *pHandle)
{
	pHandle->EncRes = MOTOR_ENC_RES;
	pHandle->Ui8ServoOn = 0;
	pHandle->Ui16ControlFrequency = 16000;	
	pHandle->u8PolePair = 4;
	pHandle->u16Offset_Ia=0x7fff;
	pHandle->u16Offset_Ib=0x7fff;
	pHandle->Pulse = pHandle->Offset_Enc;
	pHandle->DeltaPos = 0;
	pHandle->PrevDeltaPos = 0;
	pHandle->bInitPos = 0;
	pHandle->PreEncSingleTurn = 0;
	pHandle->EncSingleTurn = 0;
}


// Mr.Thinh comment		
//void UpdateDriverAndMotorParameter(uint32_t *DriverParameter, uint32_t *MotorParameter, CurrentControl_handle_t *pCurrent, VelocityControl_handle_t *pVelocity, PositionControl_handle_t *pPosition)
//{
//	//pVelocity->Speed_PI.fKp = DriverParameter[SPEED_P_GAIN];
//	//pVelocity->Speed_PI.fKi = DriverParameter[SPEED_I_GAIN];
//	//pVelocity->AccTime = DriverParameter[ACCELERATION_TIME];
//	//pVelocity->DecTime = DriverParameter[DECELERATION_TIME];
//	//pVelocity->MaxSpeed = DriverParameter[MAXIMUM_SPEED];
//	
//	//JOG->AccStep = pVelocity->MaxSpeed/SPEED_CONVERSION_FACTOR;
//	//JOG->DecStep = pVelocity->MaxSpeed/SPEED_CONVERSION_FACTOR;
//	//Init_First_Order_Lowpass_Filter(&SpeedDetecionFilter, DriverParameter[SPEED_DETECTION_FILTER_FREQUENCY], SPEED_LOOP_FREQUENCY);
//	//Init_First_Order_Lowpass_Filter(&TorqueFilter_LPF1, DriverParameter[TORQUE_FILTER_FREQUENCY], CURRENT_LOOP_FREQUENCY);
//	if(DriverParameter[CONTROL_MODE] == POSITION_CONTROL_MODE) 
//	{	//Position Control init
//		PositionControl_SetParam(pPosition, DriverParameter[POSITION_P_GAIN], -(float)MotorParameter[MOTOR_MAXIMUM_SPEED], (float)MotorParameter[MOTOR_MAXIMUM_SPEED], (float)DriverParameter[POSITION_FF_GAIN], (float)DriverParameter[POSITION_FF_FILTER]);
//	} else
//		PositionControl_Reset(pPosition);
//	
//	//	cc_Foc.Iq_Ctrl.Kp = MotorParameter[MOTOR_CURRENT_P_GAIN];
//	//	cc_Foc.Iq_Ctrl.Ki = MotorParameter[MOTOR_CURRENT_I_GAIN];
//	//	cc_Foc.Id_Ctrl.Kp = MotorParameter[MOTOR_CURRENT_P_GAIN];
//	//	cc_Foc.Id_Ctrl.Ki = MotorParameter[MOTOR_CURRENT_I_GAIN];

//	CurrentControl_SetParam(pCurrent, MotorParameter[MOTOR_CURRENT_P_GAIN], MotorParameter[MOTOR_CURRENT_I_GAIN], 32768);

//	if(((float)MotorParameter[MOTOR_RATED_CURRENT_RMS]) > DIRVER_OVER_CURRENT_THRESHOLD_AMPERE_UNIT)
//		Current_Sensor.OverCurrentThreshold = DIRVER_OVER_CURRENT_THRESHOLD_AMPERE_UNIT;
//	else
//		Current_Sensor.OverCurrentThreshold = MotorParameter[MOTOR_RATED_CURRENT_RMS];
//	//pVelocity.ConversionFactor = 60000.0f / (float)MotorParameter[MOTOR_ENCODER_RESOLUTION];
//	//DivEncoderResolution = 1/ (float)MotorParameter[MOTOR_ENCODER_RESOLUTION];
//	
//	float Velocity_OutputLimit = (int16_t)((float)MotorParameter[MOTOR_RATED_CURRENT_RMS]/ 1000);
//	VelocityControl_SetParam(pVelocity, DriverParameter[SPEED_P_GAIN], DriverParameter[SPEED_I_GAIN], Velocity_OutputLimit);
//	//JOG.PosSaturation = (float)(MotorParameter[MOTOR_ENCODER_RESOLUTION] - 1);
//}

uint16_t CheckCurrentPhaseFault(CurrentSensor_t *pHandle, float CurrentPhaseU, float CurrentPhaseV, float CurrentPhaseW)
{
	uint16_t fault = NO_FAULTS;
	if(fabsf(CurrentPhaseU) > pHandle->OverCurrentThreshold)
		fault = ERROR_OVER_CURRENT;
	else if(fabsf(CurrentPhaseV) > pHandle->OverCurrentThreshold)
		fault = ERROR_OVER_CURRENT;
	else if(fabsf(CurrentPhaseW) > pHandle->OverCurrentThreshold)
		fault = ERROR_OVER_CURRENT;
	else 
		fault = NO_ERROR;
	return fault;
}

uint16_t CheckVbusFault(Parameterhandle_t *pHandle)
{
	uint16_t fault;
	if(pHandle->fVdc > OVER_VOLTAGE_THRESHOLD_VOLT_UNIT)
		fault = ERROR_OVER_VOLTAGE;
//	else if(Handle->DCBusValue < Handle->UnderVoltThreshold)
//		fault = ERROR_UNDER_VOLTAGE;
	else
		fault = NO_ERROR;
	return fault;
}

uint16_t CheckTempFault(Parameterhandle_t *pHandle)
{
	uint16_t fault;
	if(pHandle->fTemparature > OVER_TEMPERATURE_THRESHOLD_DEGREE_UNIT)
		fault = ERROR_OVER_TEMPERATURE;
	else if(pHandle->fTemparature < OVER_TEMPERATURE_DEADBAND_DEGREE_UNIT)
		fault = NO_ERROR;
	return fault;
}

void CalibrateCurrentSensor(CurrentSensor_t *pHandle, Parameterhandle_t *param)
{
	
	if(pHandle->CalibFinish == 0)
	{
		if((pHandle->CalibCounter++) < 5000)
		{
			pHandle->Sum1 += (uint32_t)*(__IO uint16_t*)(REG_CURRENT_PHASE_U);
			pHandle->Sum2 += (uint32_t)*(__IO uint16_t*)(REG_CURRENT_PHASE_V);
		}
		else
		{
			param->u16Offset_Ia = (uint16_t)(pHandle->Sum1 / 5000);
			param->u16Offset_Ib = (uint16_t)(pHandle->Sum2 / 5000);
			pHandle->Sum1 = 0;
			pHandle->Sum2 = 0;
			pHandle->CalibFinish = 1;
			pHandle->CalibCounter = 0;
		}
	}
}

void Reset_CurrentSensor(CurrentSensor_t * pHandle)
{
	pHandle->Conversionfactor = 0;								/*Convert ADC value to current value(Ampere)*/
	pHandle->Sum1 = 0;
	pHandle->Sum2 = 0;
	pHandle->CalibFinish = 0;
	pHandle->CalibCounter = 0;
}
