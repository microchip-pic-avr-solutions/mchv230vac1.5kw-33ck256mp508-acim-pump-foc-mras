// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file estim_mras.c
 *
 * @brief This module implements MRAS(Model Reference Adaptive System ) Estimator.
 * This is a sensor-less speed observer based on motor back EMF.
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

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include <stdbool.h>

/* _Q15abs and _Q15sqrt function use */
#include <libq.h>

#include "estim_mras.h"
#include "sat_pi/sat_pi.h"

#include "general.h"

// </editor-fold>

/**
* <B> Function: void MCAPP_EstimatorMRASInit(MCAPP_MRAS_ESTIMATOR_T *)  </B>
*
* @brief Function to reset MRAS Estimator Data Structure variables.
*
* @param    pointer to the data structure containing MRAS Estimator parameters.
* @return   none.
* @example
* <CODE> MMCAPP_EstimatorMRASInit(&estimator); </CODE>
*
*/
void MCAPP_EstimatorMRASInit(MCAPP_ESTIMATOR_MRAS_T *pMRAS)
{
    pMRAS->qRhoStateVar  = 0;
    pMRAS->ImrEstimStateVar = 0;
    pMRAS->qOmegaRotorStateVar = 0;
    pMRAS->MRAS_PISTATE.integrator = 0;
    
    MCAPP_MRAS_InitLowPassFilter(&pMRAS->LPF_MRAS_omegaSyn);
    
    MCAPP_MRAS_InitLowPassFilter(&pMRAS->LPF_MRAS_omegaR);
    
    pMRAS->omegaRotor        = 0;
}


/**
* <B> Function: void MCAPP_EstimatorMRAS(MCAPP_MRAS_ESTIMATOR_T *)  </B>
*
* @brief Observer to determine rotor speed and position based on
* motor parameters and feedbacks.
*
* @param    pointer to the data structure containing MRAS Estimator parameters.
* @return   none.
* @example
* <CODE> MCAPP_EstimatorMRAS(&estimator); </CODE>
*
*/
void MCAPP_EstimatorMRAS(MCAPP_ESTIMATOR_MRAS_T *pMRAS)
{
    const MCAPP_CONTROL_T *pCtrlParam = pMRAS->pCtrlParam;
    const MC_DQ_T    *pIdq = pMRAS->pIdq; 
    const MC_DQ_T    *pVdq = pMRAS->pVdq ;  
    
    pMRAS->omegaRef    =  pCtrlParam->qVelRef;


    
    /* Reference Reactive Power Calculation */
    /*  Qref = vq*id - vd*iq */    
    pMRAS->reactivePowerRef.First_Term  = UTIL_SatShrS16(__builtin_mulss(pVdq->q, pIdq->d),15);
    pMRAS->reactivePowerRef.Second_Term = UTIL_SatShrS16(__builtin_mulss( pVdq->d, pIdq->q ),15);
    
    pMRAS->reactivePowerRef.Final_term  = pMRAS->reactivePowerRef.First_Term - pMRAS->reactivePowerRef.Second_Term;

    /* Estimated Reactive Power Calculation */
    /*  Qest = Ls*id*id*we + sigmaLs*iq*iq*we */   
    pMRAS->idSquare = __builtin_mulss(pIdq->d, pIdq->d)>>15; 
    pMRAS->idSquareLs = __builtin_mulss(pMRAS->idSquare, pMRAS->Ls)>>(15);
    pMRAS->reactivePowerEst.First_Term = UTIL_SatShrS16(__builtin_mulss(pMRAS->idSquareLs,pMRAS->omegaSyn_fil),(pMRAS->LsScale));
      
    pMRAS->iqSquare = __builtin_mulss(pIdq->q, pIdq->q)>>15;
    pMRAS->iqSquareSigmaLs  = __builtin_mulss(pMRAS->iqSquare, pMRAS->sigmaLs)>>15;
    pMRAS->reactivePowerEst.Second_Term = UTIL_SatShrS16(__builtin_mulss(pMRAS->iqSquareSigmaLs,pMRAS->omegaSyn_fil),(pMRAS->sigmaLsScale));
    
    pMRAS->reactivePowerEst.Final_term  = pMRAS->reactivePowerEst.First_Term + pMRAS->reactivePowerEst.Second_Term ;


    if ( (pIdq->q  < 0) )
    {
        MCAPP_ControllerPIUpdate(pMRAS->reactivePowerEst.Final_term, pMRAS->reactivePowerRef.Final_term, 
            &pMRAS->MRAS_PISTATE, MCAPP_SAT_NONE, &pMRAS->omegaPetrub,0); 
    }
    else
    {
        MCAPP_ControllerPIUpdate(pMRAS->reactivePowerRef.Final_term, pMRAS->reactivePowerEst.Final_term, 
            &pMRAS->MRAS_PISTATE, MCAPP_SAT_NONE, &pMRAS->omegaPetrub,0);
    }
    pMRAS->omegaRotor         = pMRAS->omegaRef + pMRAS->omegaPetrub;
    /* Filter the estimated  rotor velocity using a first order low-pass filter */
    const int16_t Omegadiff = (int16_t) (pMRAS->omegaRotor - pMRAS->omegaRotorFil);
    pMRAS->qOmegaRotorStateVar += __builtin_mulss(Omegadiff, pMRAS->qOmegaFiltConst);
    pMRAS->omegaRotorFil = (int16_t) (pMRAS->qOmegaRotorStateVar >> 15);  
    
    
    /* Estimate the Magnetizing Current Imr = (Id/(TrS + 1))*/
    const int16_t Imrdiff = (int16_t) (pCtrlParam->qIdRef - pMRAS->ImrEstim);
    pMRAS->ImrEstimStateVar += __builtin_mulss(Imrdiff, pMRAS->qImrEstimFilterK);
    pMRAS->ImrEstim = (int16_t) (pMRAS->ImrEstimStateVar >> 15);
    
    /* Estimate the slip value wslip = ((1/Tr) * (iq/imr))*/
    const int32_t iqTr = __builtin_mulss(pMRAS->InvTr, pIdq->q);
    if((pMRAS->ImrEstim  > 0))
    {    
        pMRAS->omegaSlip =  __builtin_divsd(iqTr, pMRAS->ImrEstim);
    }
    else{
        pMRAS->omegaSlip = 0;
    }
               
    pMRAS->omegaSyn = pMRAS->omegaRotor + pMRAS->omegaSlip;
    
    pMRAS->LPF_MRAS_omegaSyn.X   =  pMRAS->omegaSyn ;
    pMRAS->omegaSyn_fil  =  MCAPP_MRAS_LowPassFilter(&pMRAS->LPF_MRAS_omegaSyn); 
    
    pMRAS->qRhoStateVar += __builtin_mulss(pMRAS->omegaSyn_fil, pMRAS->qDeltaT);
    pMRAS->qRho         = (int16_t) (pMRAS->qRhoStateVar >> 15);
}



/*
 * Bilinear Implementation of Low Pass Filter
 */
int16_t MCAPP_MRAS_LowPassFilter(LPF *lpf)
{
	lpf->Y = UTIL_SatShrS16(__builtin_mulss(lpf->Y_prev,lpf->Y_Coeff),15) 
            + UTIL_SatShrS16(__builtin_mulss(lpf->X_Coeff,(lpf->X)),15)
            + UTIL_SatShrS16(__builtin_mulss(lpf->X_Coeff,(lpf->X_prev)),15);
	lpf->Y_prev = lpf->Y;
	lpf->X_prev = lpf->X;
	return(lpf->Y);
}

void MCAPP_MRAS_InitLowPassFilter(LPF *lpf)
{
	lpf->Y         = 0;
	lpf->Y_prev    = 0;
	lpf->X         = 0;
	lpf->X_prev    = 0;
}




