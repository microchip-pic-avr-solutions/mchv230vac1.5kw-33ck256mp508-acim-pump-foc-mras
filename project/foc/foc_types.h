// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
* @file foc_types.h
*
* @brief This module has variable type definitions of data structure
* holding foc parameters and enumerated constants.
*
* Component: FOC
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

#ifndef __FOC_TYPES_H
#define __FOC_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include <stdbool.h>

#include "foc_control_types.h"
#include "estim_interface.h"
#include "estim_mras.h"
#include "id_ref.h"
#include "motor_control.h"
#include "motor_params.h"
#include "sat_pi/sat_pi.h"
// </editor-fold>

// <editor-fold defaultstate="expanded" desc="ENUMERATED CONSTANTS ">

typedef enum
{
    FOC_INIT = 0,              /* Initialize control parameters */
    FOC_OPEN_LOOP = 1,         /* Open Loop */
    FOC_CLOSE_LOOP = 2,        /* Closed Loop */
    FOC_FAULT = 3,             /* Motor is in Fault */

}FOC_CONTROL_STATE_T;

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="VARIABLE TYPE DEFINITIONS ">

typedef struct
{
    int16_t
        *pIa,               /* Pointer for Ia */
        *pIb,               /* Pointer for Ib */
        *pVdc,              /* Pointer for Vdc */
        faultStatus,        /* Fault Status */
        focState;           /* FOC State */       
    
    
    MCAPP_PISTATE_T
        piIq,               /* Parameters for PI Q axis controllers */
        piId,               /* Parameters for PI D axis controllers */
        piSpeed;                 /* Parameters for PI Speed controllers */    
    
    MC_SINCOS_T
        sincosTheta;        /* Sincos parameters */
    
    MC_DQ_T
        vdq,                /* Vdq */
        idq;                /* Idq */
    
    MC_ABC_T
        iabc,               /* Iabc */
        vabc,               /* Vabc */
        vabcScaled,         /* Vabc scaled for MC_CalculateSpaceVectorPhaseShifted_Assembly */
        vabcCompDC;         /* Vabc dc link compensated */
    
    MC_ALPHABETA_T
        ialphabeta,         /* IalphaBeta */
        valphabeta;         /* Duty_alphaBeta */
    
    MC_DUTYCYCLEOUT_T
        *pPWMDuty;            /* Duties out */
    
    MCAPP_ESTIMATOR_MRAS_T
        estimMRAS;         /* MRAS Estimator Structure */

        
    MCAPP_ESTIMATOR_T
        estimInterface;     /* Estimator Interface Structure */

    MCAPP_IDREFGEN_T
        idRefGen;           
    
    MCAPP_CONTROL_T
        ctrlParam;          /* Parameters for control references */

    MCAPP_MOTOR_T
        *pMotor;            /* Pointer for Motor Parameters */
    
    
    uint16_t pwmPeriod;     /* PWM Period */

}MCAPP_FOC_T;

typedef enum
{
    STALL_DETECTED = 1,
    FOC_OVERCURRENT_FAULT = 2,
}MCAPP_FAULT_STATUS_T;

// </editor-fold>

#ifdef __cplusplus
}
#endif

#endif /* end of __FOC_TYPES_H */
