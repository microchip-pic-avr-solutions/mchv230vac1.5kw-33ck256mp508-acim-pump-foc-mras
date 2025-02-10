// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file mc1_init.h
 *
 * @brief This module initializes data structure holding motor control
 * parameters required to run motor 1 using field oriented control.
 * In this application to initialize variable required to run the application.
 *
 * Component: APPLICATION (Motor Control 1 - mc1)
 *
 */
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Disclaimer ">

/*******************************************************************************
* SOFTWARE LICENSE AGREEMENT
* 
* © [2024] Microchip Technology Inc. and its subsidiaries
* 
* Subject to your compliance with these terms, you may use this Microchip 
* software and any derivatives exclusively with Microchip products. 
* You are responsible for complying with third party license terms applicable to
* your use of third party software (including open source software) that may 
* accompany this Microchip software.
* 
* Redistribution of this Microchip software in source or binary form is allowed 
* and must include the above terms of use and the following disclaimer with the
* distribution and accompanying materials.
* 
* SOFTWARE IS "AS IS." NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY,
* APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,
* MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL 
* MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR 
* CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO
* THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE 
* POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY
* LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL
* NOT EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR THIS
* SOFTWARE
*
* You agree that you are solely responsible for testing the code and
* determining its suitability.  Microchip has no obligation to modify, test,
* certify, or support the code.
*
*******************************************************************************/
// </editor-fold>

#ifndef __MC1_INIT_H
#define __MC1_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include <stdbool.h>

#include "measure.h"
#include "motor_params.h"
#include "foc.h"
#include "application/application.h"
    
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="VARIABLE TYPE DEFINITIONS ">

typedef struct
{
    int16_t
        appState,                   /* Application State */
        runCmd,                     /* Run command for motor */
        runCmdBuffer,               /* Run command buffer for validation */
        qTargetVelocity,            /* Target motor Velocity */
        qMaxSpeedFactor;            /* Maximum speed to peak speed ratio */
    
    MCAPP_MEASURE_T
        motorInputs;
    
    MCAPP_MOTOR_T
        motor;
    
    MCAPP_CONTROL_SCHEME_T
        controlScheme;              /* Motor Control parameters */
    
    MCAPP_LOAD_T
        load;                       /* Load parameters */
    
    MC_DUTYCYCLEOUT_T
        PWMDuty;
    
    MCAPP_MEASURE_T *pMotorInputs;
    MCAPP_MOTOR_T *pMotor;
    MCAPP_CONTROL_SCHEME_T *pControlScheme;
    MCAPP_LOAD_T *pLoad;
    MC_DUTYCYCLEOUT_T *pPWMDuty;
        
    /* Function pointers for motor inputs */    
    void (*MCAPP_InputsInit) (MCAPP_MEASURE_T *);
    void (*MCAPP_MeasureOffset) (MCAPP_MEASURE_T *);
    void (*MCAPP_GetProcessedInputs) (MCAPP_MEASURE_T *);
    int16_t (*MCAPP_IsOffsetMeasurementComplete) (MCAPP_MEASURE_T *);
    void (*HAL_MotorInputsRead) (MCAPP_MEASURE_T *);
    
    /* Function pointers for control scheme */
    void (*MCAPP_ControlSchemeInit) (MCAPP_CONTROL_SCHEME_T *);
    void (*MCAPP_ControlStateMachine) (MCAPP_CONTROL_SCHEME_T *);
    
    /* Function pointers for load */
    void (*MCAPP_LoadStateMachine) (MCAPP_LOAD_T *);
    void (*MCAPP_LoadInit) (MCAPP_LOAD_T *);
    
    void (*MCAPP_LoadStartTransition) (MCAPP_CONTROL_SCHEME_T *, 
                                        MCAPP_LOAD_T *);
    void (*MCAPP_LoadStopTransition) (MCAPP_CONTROL_SCHEME_T *, 
                                        MCAPP_LOAD_T *);
    
    int16_t (*MCAPP_IsLoadReadyToStart) (MCAPP_LOAD_T *);
    int16_t (*MCAPP_IsLoadReadyToStop) (MCAPP_LOAD_T *);
    
    /* Function pointers for motor outputs */
    void (*HAL_PWMSetDutyCycles)(MC_DUTYCYCLEOUT_T *);
    void (*HAL_PWMEnableOutputs) (void);
    void (*HAL_PWMDisableOutputs) (void);
    void (*MCAPP_HALSetVoltageVector) (int16_t);

}MC1APP_DATA_T;

// </editor-fold>
    
// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

void MCAPP_MC1ParamsInit(MC1APP_DATA_T *);

// </editor-fold>


#ifdef __cplusplus
}
#endif

#endif /* end of __MC1_INIT_H */
