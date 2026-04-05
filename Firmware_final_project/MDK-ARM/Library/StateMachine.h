#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>

// Define system states
typedef enum {
    IDLE = 0,           // Waiting for command, PWM off
    OFFSET_CALIB,       // Current sensor zero-offset calibration
    ENCODER_ALIGN,      // Encoder electrical-zero alignment and offset capture
    START,              // Soft start sequence
    RUN,                // Main execution (FOC, Tuning, JOG)
    STOP,               // Stop motor, reset PI controllers
    FAULT_NOW,          // Critical fault detected, immediate PWM stop
    FAULT_OVER          // Fault cleared, waiting for reset command
} State_t;

// State machine data structure
typedef struct {
    State_t bState;       // Current state
    State_t bPreState;    // Previous state for transition tracking
    uint32_t FaultCode;   // Active fault codes bitmask
} StateMachine_t;

// Function prototypes
void STM_Init(StateMachine_t *sm);
void STM_NextState(StateMachine_t *sm, State_t nextState);
State_t STM_GetState(StateMachine_t *sm);
void STM_FaultProcessing(StateMachine_t *sm, uint32_t faultCode, uint32_t clearMask);

#endif // STATE_MACHINE_H