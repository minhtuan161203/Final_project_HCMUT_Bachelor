#include "StateMachine.h"

// Initialize the state machine to default values
void STM_Init(StateMachine_t *sm) {
    if (sm == 0) return;
    
    sm->bState = IDLE;
    sm->bPreState = IDLE;
    sm->FaultCode = 0; // 0 indicates no active faults
}

// Return the current state
State_t STM_GetState(StateMachine_t *sm) {
    return sm->bState;
}

// Safely transition to a new state
void STM_NextState(StateMachine_t *sm, State_t nextState) {
    if (sm == 0) return;
    
    // Prevent leaving fault states unless explicitly commanded to IDLE or STOP
    if (sm->bState == FAULT_NOW || sm->bState == FAULT_OVER) {
        if (nextState != IDLE && nextState != STOP) {
            return; // Ignore invalid transitions during a fault
        }
    }

    // Update state history and set new state
    sm->bPreState = sm->bState;
    sm->bState = nextState;
}

// Process hardware and software faults
void STM_FaultProcessing(StateMachine_t *sm, uint32_t faultCode, uint32_t clearMask) {
    if (sm == 0) return;

    // Update active faults
    sm->FaultCode |= faultCode; 
    
    // Clear resolved faults based on mask
    sm->FaultCode &= clearMask; 

    // Force FAULT_NOW if any error exists
    if (sm->FaultCode != 0) { 
        if (sm->bState != FAULT_NOW && sm->bState != FAULT_OVER) {
            STM_NextState(sm, FAULT_NOW);
        }
    } 
    // Transition to FAULT_OVER when errors are cleared but system hasn't reset
    else if (sm->FaultCode == 0 && sm->bState == FAULT_NOW) {
        STM_NextState(sm, FAULT_OVER);
    }
}