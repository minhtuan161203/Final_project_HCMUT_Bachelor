#include "PWM.h"
#include "define.h"
#include "stm32f4xx_hal.h"
#include <math.h> // Required for sinf()

extern TIM_HandleTypeDef htim8;

// --- V/f Control Constants ---
#define NOMINAL_FREQ    50.0f    // Rated motor frequency (Hz)
#define NOMINAL_V       0.90f    // Max duty cycle limit (0.0 to 1.0)
#define V_OFFSET        0.05f    // Voltage boost for low-speed torque

// --- Global State Variables ---
static float current_phase = 0.0f; 

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
    if(DutyCycle_U > 0.90f) DutyCycle_U = 0.90f;
    if(DutyCycle_V > 0.90f) DutyCycle_V = 0.90f;
    if(DutyCycle_W > 0.90f) DutyCycle_W = 0.90f;
    
    // Setting Compare Registers
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, DutyCycle_U * FULL_PWM_PERIOD);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, DutyCycle_V * FULL_PWM_PERIOD);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, DutyCycle_W * FULL_PWM_PERIOD);
}

/**
 * @brief V/f Control: Generates 3-phase sine waves based on target frequency.
 * @param TargetFreq: Desired frequency (Hz)
 * @param SamplingTime: The loop time (e.g., 0.0001 for 10kHz)
 */
PWM_Phases_t Control_V_over_F(float TargetFreq, float SamplingTime, float TargetVol)
{
    float amplitude;
		PWM_Phases_t output;

    // 1. Calculate Voltage Amplitude (V = k*f + offset)
    if (TargetFreq <= 0.1f) {
        amplitude = 0.0f;
    } else {
        amplitude = TargetVol;
        if (amplitude > NOMINAL_V) amplitude = NOMINAL_V;
    }

    // 2. Accumulate Phase Angle (Theta = Integral of frequency)
    // Angle increases by (2*PI * f * dt) every loop
    current_phase += 2.0f * PI * 100 * SamplingTime;

    // Keep the angle between 0 and 2*PI
    if (current_phase > 2.0f * PI) {
        current_phase -= 2.0f * PI;
    }

    // 3. Generate 3-phase Sine Waves (120-degree shifts)
    // We add 0.5f because PWM duty cycle must be positive (0.0 to 1.0)
    output.U = 0.5f + (amplitude / 2.0f) * sinf(current_phase);
    output.V = 0.5f + (amplitude / 2.0f) * sinf(current_phase - (2.0f * PI / 3.0f));
    output.W = 0.5f + (amplitude / 2.0f) * sinf(current_phase + (2.0f * PI / 3.0f));

    // 4. Update the Hardware
    GeneratePWM(output.U, output.V, output.W);
		return output;
}
