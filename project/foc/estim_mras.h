// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file estim_mras.h
 *
 * @brief This module implements MRAS(Model Reference Adaptive System ) Estimator.
 *
 * Component: ESTIMATOR
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

#ifndef __ESTIM_MRAS_H
#define __ESTIM_MRAS_H

#ifdef __cplusplus
    extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include <stdbool.h>

#include "motor_control.h"
#include "measure.h"
#include "foc_control_types.h"
#include "motor_params.h"
        
#include "sat_pi/sat_pi.h"
        
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="VARIABLE TYPE DEFINITIONS">

        
/**
 * State variables for Reactive Power 
 */
typedef struct
{
    int16_t     First_Term;
    int16_t     Second_Term;
    int16_t     Final_term;
} MRAS_ESTIMATOR_Reactive_Power;


/**
 * State variables for Low Pass Filter
 */
typedef struct
{
    int16_t Y;                            // Output
    int16_t Y_prev;                       // Previous Value of Output
    int16_t X;                            // Output
    int16_t X_prev;                       // Previous Value of Input
    int16_t X_Coeff;                      // Input Coeff
    int16_t Y_Coeff;                      // Output Coeff
}LPF;



/**
 * State variables for MRAS Estimator 
 */
typedef struct tag_ESTIMATOR_MRAS
{
    /*
     * Runtime-adjustable parameters
     *
     * These values are typically set once, at startup,
     * but may be adjusted using real-time diagnostic tools.
     */ 
    int16_t                                 sigmaLs;
    int16_t                                 Ls;
    int16_t                                 sigmaLsScale;
    int16_t                                 LsScale;
    int16_t                                 idSquare;
    int16_t                                 iqSquare;
    int16_t                                 idSquareLs;
    int16_t                                 iqSquareSigmaLs;
    
    int16_t                                 omegaPetrub;
    int16_t                                 omegaRef;
    int16_t                                 omegaSyn;
    int16_t                                 omegaSyn_fil;
    int16_t                                 omegaSlip;
    int16_t                                 omegaRotor;          /** Estimated rotor speed */   
    int16_t                                 omegaRotorFil;       /** Estimated rotor speed filtered */   
    int16_t                                 qOmegaFiltConst;    /* Filter constant for Estimated rotor speed */
    int32_t                                 qOmegaRotorStateVar;/* State Variable for Estimated rotor speed */
    int16_t                                 qRho;                /** electrical theta */
    int32_t                                 qRhoStateVar;             /** electrical theta 32 bit*/
    int16_t                                 qDeltaT;
    int16_t                                 ImrEstim;    
    int16_t                                 InvTr;            /*inverse Rotor Time Constant (Tr)*/
    int16_t                                 qImrEstimFilterK; /* Filter constant for Estimated Imr */
    
    int32_t                                 ImrEstimStateVar;
    
    MC_DQ_T                       isdq;                /** Previous D-Q Currents **/
    MC_DQ_T                       vdq; 

    MRAS_ESTIMATOR_Reactive_Power      reactivePowerRef;   /** Reference Reactive Power **/
    MRAS_ESTIMATOR_Reactive_Power      reactivePowerEst;   /** Estimated Reactive Power **/
    MCAPP_PISTATE_T                     MRAS_PISTATE;       /** MRAS PI Controller */
    LPF                                LPF_MRAS_omegaR;           /** MRAS LPF Filter for */
    LPF                                LPF_MRAS_omegaSyn;          /** MRAS LPF Filter */
   

    
    const MC_DQ_T    *pIdq;  /*  DQ Current */
    const MC_DQ_T    *pVdq;  /*  DQ Voltage */
    const MCAPP_CONTROL_T *pCtrlParam;
    const MC_ALPHABETA_T *pIAlphaBeta;
    const MC_ALPHABETA_T *pVAlphaBeta;
    const MCAPP_MOTOR_T *pMotor;
} MCAPP_ESTIMATOR_MRAS_T;


        

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

void MCAPP_EstimatorMRASInit (MCAPP_ESTIMATOR_MRAS_T *);
void MCAPP_EstimatorMRAS (MCAPP_ESTIMATOR_MRAS_T *);

int16_t MCAPP_MRAS_LowPassFilter(LPF *);
void MCAPP_MRAS_InitLowPassFilter(LPF *);



// </editor-fold>

#ifdef __cplusplus
    }
#endif

#endif /* end of __ESTIM_MRAS_H */
