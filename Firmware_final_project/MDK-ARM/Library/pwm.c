#include "PWM.h"
#include "define.h"
#include "stm32f4xx_hal.h"
#include <math.h> // Required for sinf()

extern TIM_HandleTypeDef htim8;

// --- V/f Control Constants ---
#define NOMINAL_FREQ    50.0f    // Rated motor frequency (Hz)
#define NOMINAL_V       0.90f    // Max duty cycle limit (0.0 to 1.0)
#define V_OFFSET        0.05f    // Voltage boost for low-speed torque
#define VF_SAMPLING_TIME_S (1.0f / CURRENT_LOOP_FREQUENCY)
#define TWO_PI_F  (2.0f * PI)
#define SQRT3_BY_2_F 0.86602540378f

// --- Global State Variables ---
static float current_phase = 0.0f; 

void ResetControl_V_over_F(void)
{
    current_phase = 0.0f;
}

void SwitchOnPWM(void)
{
    HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);

    HAL_TIMEx_PWMN_Start(&htim8,TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim8,TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim8,TIM_CHANNEL_3);
}

void SwitchOffPWM(void)
{
    HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_3);

    HAL_TIMEx_PWMN_Stop(&htim8,TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim8,TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim8,TIM_CHANNEL_3);
}

void GeneratePWM(float DutyCycle_U, float DutyCycle_V, float DutyCycle_W)
{
    // Safety Clamping
    if(DutyCycle_U < 0.0f) DutyCycle_U = 0.0f;
    if(DutyCycle_V < 0.0f) DutyCycle_V = 0.0f;
    if(DutyCycle_W < 0.0f) DutyCycle_W = 0.0f;
    if(DutyCycle_U > 0.90f) DutyCycle_U = 0.90f;
    if(DutyCycle_V > 0.90f) DutyCycle_V = 0.90f;
    if(DutyCycle_W > 0.90f) DutyCycle_W = 0.90f;
    
    // Setting Compare Registers
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, DutyCycle_U * FULL_PWM_PERIOD);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, DutyCycle_V * FULL_PWM_PERIOD);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, DutyCycle_W * FULL_PWM_PERIOD);
}

/**
 * @brief Open-loop 3-phase generator used by the higher-level V/F mode.
 * @param TargetFreq: Electrical output frequency in Hz.
 * @param TargetVol: Normalized voltage / modulation command (0.0 to 0.90).
 */
PWM_Phases_t Control_V_over_F(float TargetFreq,  float TargetVol)
{
    float limited_frequency = TargetFreq;
    float modulation = TargetVol;
    float sin_theta;
    float cos_theta;
    PWM_Phases_t output;

    if (limited_frequency > NOMINAL_FREQ) {
        limited_frequency = NOMINAL_FREQ;
    } else if (limited_frequency < -NOMINAL_FREQ) {
        limited_frequency = -NOMINAL_FREQ;
    }

    if (modulation < 0.0f) {
        modulation = 0.0f;
    } else if (modulation > NOMINAL_V) {
        modulation = NOMINAL_V;
    }

    if ((fabsf(limited_frequency) <= 0.1f) || (modulation <= 0.001f)) {
        output.U = 0.5f;
        output.V = 0.5f;
        output.W = 0.5f;
        GeneratePWM(output.U, output.V, output.W);
        return output;
    }

    /* FPGA interrupt drives this function at 16 kHz, so phase is integrated
       from the measured ISR period instead of a magic literal. */
    current_phase += TWO_PI_F * limited_frequency * VF_SAMPLING_TIME_S;

    if (current_phase >= TWO_PI_F) {
        current_phase -= TWO_PI_F;
    } else if (current_phase < 0.0f) {
        current_phase += TWO_PI_F;
    }

    /* One sinf/cosf pair is cheaper than three independent sinf() calls
       inside the ISR and keeps the phase relationship exact. */
    sin_theta = sinf(current_phase);
    cos_theta = cosf(current_phase);

    output.U = 0.5f + 0.5f * modulation * sin_theta;
    output.V = 0.5f + 0.5f * modulation *
        ((-0.5f * sin_theta) - (SQRT3_BY_2_F * cos_theta));
    output.W = 0.5f + 0.5f * modulation *
        ((-0.5f * sin_theta) + (SQRT3_BY_2_F * cos_theta));

    GeneratePWM(output.U, output.V, output.W);
    return output;
}

