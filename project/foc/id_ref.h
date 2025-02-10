// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file id_ref.h
 *
 * @brief This module implements Id current reference generation of 
 * Induction Motors.
 *
 * Component: ID REFERENCE GENERATION
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

#ifndef __ID_REF_H
#define __ID_REF_H

#ifdef __cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>

#include "foc_control_types.h"
#include "measure.h"
#include "estim_interface.h"
#include "motor_control.h"
#include "motor_params.h"
#include "estim_mras.h"    
#include "sat_pi/sat_pi.h"

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="VARIABLE TYPE DEFINITIONS ">

typedef struct
{
    int16_t
        IdRef,              /* Id Current reference */
        IdRefFilt,          /* Filtered Id Current reference */
        IdRefFiltConst,     /* Filter constant for Id */
        IdRefMin,           /* Lower limit on IdRef */
        voltageMag,         /* Voltage vector magnitude */
        voltageMagRef;      /* Voltage vector magnitude reference */
            
    int32_t
        IdRefFiltStateVar;  /* Accumulation variable for IdRef filter */
    
    MCAPP_PISTATE_T FWeakPI;

    MCAPP_CONTROL_T *pCtrlParam;
    const MC_DQ_T *pVdq;
    const MCAPP_MOTOR_T *pMotor;

} MCAPP_FWEAK_VOLTAGE_FB_T;


typedef struct
{
    int16_t
        FdweakStartSpeed,
        IdRef,              /* Id Current reference */
        IdRefMin,           /* Lower limit on IdRef */   
        fdWeakStartSpeed;   /* Field weakening start speed */

    int32_t 
        FWeakConstant;     

    MCAPP_CONTROL_T *pCtrlParam;
    const MCAPP_MOTOR_T *pMotor;

} MCAPP_FWEAK_TYPE1_T;


typedef struct
{
    int16_t
        InvLoRatio,
        LoRatioByTwo,
        paramUpdateCoef1,
        paramUpdateCoef1Qvalue,
        paramUpdateCoef2,
        Nom_InvLmSqrbyLr,
        Nom_InvTr,
        Nom_Ls,
        Nom_sigmaLs;
    
    MCAPP_CONTROL_T *pCtrlParam;
    MCAPP_MOTOR_T *pMotor;
    MCAPP_ESTIMATOR_MRAS_T *pEstimMRAS;
    
} MCAPP_UPDATE_PARAMS_T;


typedef struct
{
    int16_t 
        IdRefFilt;          /* Filtered Id Current reference */
    
    MCAPP_UPDATE_PARAMS_T 
        UpdateParms;
    
    MCAPP_FWEAK_TYPE1_T 
        fWeakType1;              /* Pointer for Flux Weakening type 1*/
    
    MCAPP_FWEAK_VOLTAGE_FB_T
        fWeakType2;             /* Voltage feedback based Flux Weakening Structure */
    
    MCAPP_FWEAK_TYPE1_T 
        *pFWeakType1;           /* Pointer for Flux Weakening  type 1*/
        
    MCAPP_FWEAK_VOLTAGE_FB_T 
        *pFWeakType2;          /* Pointer for Voltage feedback based Flux Weakening */
    
    MCAPP_UPDATE_PARAMS_T 
        *pUpdateParms;
    
} MCAPP_IDREFGEN_T;


// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">
void MCAPP_IdRefGenerationInit(MCAPP_IDREFGEN_T *);
void MCAPP_IdRefGeneration(MCAPP_IDREFGEN_T *);

// </editor-fold>

#ifdef __cplusplus
}
#endif

#endif /* end of __ID_REF_H */
