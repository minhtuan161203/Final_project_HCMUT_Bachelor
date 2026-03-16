#ifndef	__PWM_H__
#define __PWM_H__

#ifdef	__cplusplus
extern "C" {
#endif /*__cplusplus*/

#include "stdint.h"	
typedef struct {
    float U;
    float V;
    float W;
} PWM_Phases_t;
	
void SwitchOnPWM(void);
void SwitchOffPWM(void);
void GeneratePWM(float DutyCycle_U, float DutyCycle_V, float DutyCycle_W);
PWM_Phases_t Control_V_over_F(float TargetFreq, float TargetVol);
#ifdef __cplusplus
}
#endif	/*__cplusplus*/

#endif /*__pwm_h*/
