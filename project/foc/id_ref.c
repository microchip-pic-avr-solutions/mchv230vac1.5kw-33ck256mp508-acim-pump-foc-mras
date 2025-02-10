// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file id_ref.c
 *
 * @brief This module implements field weakening algorithms to 
 * generate id current reference required for extended speed operation of 
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

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>

/* _Q15abs and _Q15sqrt function use */
#include <libq.h>

#include "id_ref.h"
#include "general.h"

#include "sat_pi/sat_pi.h"

#include "estim_mras.h"

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="DEFINITIONS/MACROS ">
#undef ID_REFERNCE_FILTER_ENABLE /* Id reference filter enable flag */
#define INVLORATIO_MAX 32765
#define INVLORATIO_MIN 16385
#define Q14_1   16382
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="STATIC FUNCTIONS ">
static void MCAPP_FluxControlVoltFeedback(MCAPP_FWEAK_VOLTAGE_FB_T *);
static void MCAPP_FluxControlInvSpeed(MCAPP_FWEAK_TYPE1_T *);
static void MCAPP_FWeakUpdateMotorParameters(MCAPP_UPDATE_PARAMS_T *);
static void MCAPP_FluxControlVoltFeedbackInit(MCAPP_FWEAK_VOLTAGE_FB_T *);
// </editor-fold>

/**
* <B> Function: MCAPP_FdWeakInit(MCAPP_FWEAK_VOLTAGE_FB_T * )  </B>
*
* @brief Function to reset variables used for Field Weakening Control.
*        . 
*
* @param Pointer to the data structure containing Field Weakening Control
*        parameters.
* @return   none.
* @example
* <CODE> MCAPP_FdWeakInit(&fieldWeak); </CODE>
*
*/
void  MCAPP_IdRefGenerationInit(MCAPP_IDREFGEN_T *pIdRefGen)
{ 
    MCAPP_FWEAK_VOLTAGE_FB_T *pFWeakType2 = pIdRefGen->pFWeakType2;

    MCAPP_FluxControlVoltFeedbackInit(pFWeakType2);
    
    pIdRefGen->pFWeakType2->IdRefFiltStateVar = 0;
    pIdRefGen->pFWeakType2->IdRef = 0;
}


/**
* <B> Function: MCAPP_FluxControlVoltFeedbackInit(MCAPP_FWEAK_VOLTAGE_FB_T * )  </B>
*
* @brief Function to reset variables used for Field Weakening Control.
*        .
*
* @param Pointer to the data structure containing Field Weakening Control
*        parameters.
* @return   none.
* @example
* <CODE> MCAPP_FluxControlVoltFeedbackInit(&fieldWeak); </CODE>
*
*/
static void  MCAPP_FluxControlVoltFeedbackInit(MCAPP_FWEAK_VOLTAGE_FB_T *pFdWeak)
{
    const MCAPP_MOTOR_T *pMotor = pFdWeak->pMotor;
    
    pFdWeak->FWeakPI.outMax = pMotor->qNominalMagCurrent ;
    pFdWeak->FWeakPI.outMin = pMotor->qMinMagCurrent;
    pFdWeak->FWeakPI.integrator = (((int32_t)pMotor->qNominalMagCurrent)<<16);;
}



/**
* <B> Function: MCAPP_IdReferenceGeneration(MCAPP_IDREFGEN_T * )  </B>
*
* @brief Function implementing Field Weakening Control
*
* @param Pointer to the data structure containing Field Weakening Control
*        parameters.
* @return   none.
* @example
* <CODE> MCAPP_IdReferenceGeneration(&idRefGen); </CODE>
*
*/
void MCAPP_IdRefGeneration(MCAPP_IDREFGEN_T *pIdRefGen)
{
    MCAPP_FWEAK_VOLTAGE_FB_T *pFWeakType2 = pIdRefGen->pFWeakType2;
    MCAPP_FWEAK_TYPE1_T *pFWeakType1 = pIdRefGen->pFWeakType1;
    MCAPP_UPDATE_PARAMS_T *pUpdateParms = pIdRefGen->pUpdateParms;
    /* Calculate Flux Weakening Control current */

    MCAPP_FluxControlVoltFeedback(pFWeakType2);
    MCAPP_FluxControlInvSpeed(pFWeakType1); 
    
    MCAPP_FWeakUpdateMotorParameters(pUpdateParms);
}

/**
* <B> Function: MCAPP_FluxControlVoltFeedback(MCAPP_FWEAK_VOLTAGE_FB_T * )  </B>
*
* @brief Function implementing Field Weakening Control
*
* @param Pointer to the data structure containing Field Weakening Control
*        parameters.
* @return   none.
* @example
* <CODE> MCAPP_FluxControlVoltFeedback(&fieldWeak); </CODE>
*
*/

static void MCAPP_FluxControlVoltFeedback(MCAPP_FWEAK_VOLTAGE_FB_T *pFdWeak)
{    
    const MC_DQ_T *pVdq         = pFdWeak->pVdq;
    const MCAPP_MOTOR_T *pMotor = pFdWeak->pMotor;
    MCAPP_CONTROL_T *pCtrlParam = pFdWeak->pCtrlParam;

    int16_t vdSqr, vqSqr, IdRefOut; 

    /* Compute voltage vector magnitude */
    vdSqr  = (int16_t)(__builtin_mulss(pVdq->d, pVdq->d) >> 15);
    vqSqr  = (int16_t)(__builtin_mulss(pVdq->q, pVdq->q) >> 15);
    pFdWeak->voltageMag = _Q15sqrt(vdSqr+vqSqr);
    
    if((pCtrlParam->qVelRef > (pMotor->qNominalSpeed>>1)))
    { 
        /* Compute PI output: pFdWeak->IdRef */
        MCAPP_ControllerPIUpdate(pFdWeak->voltageMagRef, pFdWeak->voltageMag, 
            &pFdWeak->FWeakPI, MCAPP_SAT_NONE, &IdRefOut, pFdWeak->voltageMagRef); 
    }
    else
    {
        IdRefOut = pMotor->qNominalMagCurrent - pCtrlParam->qIdRefOffset;
        if(pCtrlParam->qIdRefOffset > 0){
            pCtrlParam->qIdRefOffset--;
        }
        else if(pCtrlParam->qIdRefOffset < 0){
            pCtrlParam->qIdRefOffset++;
        }
        
        /* Reset PI integrator to Nominal Idrefernce value for smooth transition
         * when switching to PI output computation. */
        MCAPP_ControllerPIReset(&pFdWeak->FWeakPI, pMotor->qNominalMagCurrent);
    }

    /*Filter for the FW Idref current*/
#ifdef ID_REFERNCE_FILTER_ENABLE
    pFdWeak->IdRefFiltStateVar +=
                    __builtin_mulss((IdRefOut - pFdWeak->IdRefFilt),
                                        pFdWeak->IdRefFiltConst);
    pFdWeak->IdRefFilt = (int16_t)(pFdWeak->IdRefFiltStateVar >> 15);
    pFdWeak->IdRef = pFdWeak->IdRefFilt;
#else
    pFdWeak->IdRef = IdRefOut;
#endif
}



/**
* <B> Function: MCAPP_FluxControlInvSpeed(MCAPP_FWEAK_TYPE1_T * )  </B>
*
* @brief Function implementing Field Weakening Control with 
* conventional inverse proportion to speed method 
*
* @param Pointer to the data structure containing Field Weakening Control
*        parameters.
* @return   none.
* @example
* <CODE> MCAPP_FluxControlInvSpeed(&fieldWeak); </CODE>
*
*/

static void MCAPP_FluxControlInvSpeed(MCAPP_FWEAK_TYPE1_T *pFdWeak)
{    
    const MCAPP_MOTOR_T *pMotor = pFdWeak->pMotor;
    MCAPP_CONTROL_T *pCtrlParam = pFdWeak->pCtrlParam;

    int16_t IdReference ; 
    if( pCtrlParam->qVelRef > (pMotor->qNominalSpeed>>1) )
    {
        /* imr_ref = Imr_ref*wm_rated/wm_ref; */
        pFdWeak->FWeakConstant = __builtin_mulss(pMotor->qNominalMagCurrent, pFdWeak->FdweakStartSpeed);
        IdReference = (int16_t)(__builtin_divsd(pFdWeak->FWeakConstant, pCtrlParam->qVelRef));
        
        pFdWeak->IdRef = IdReference;
        if(IdReference < pMotor->qMinMagCurrent)
        {
            pFdWeak->IdRef = pMotor->qMinMagCurrent;
        }
        else if(IdReference > pMotor->qNominalMagCurrent)
        {
            pFdWeak->IdRef = pMotor->qNominalMagCurrent;
        }
    }
    else
    {
        pFdWeak->IdRef = pMotor->qNominalMagCurrent - pCtrlParam->qIdRefOffset;
        
        if(pCtrlParam->qIdRefOffset > 0){
            pCtrlParam->qIdRefOffset--;
        }
        else if(pCtrlParam->qIdRefOffset < 0){
            pCtrlParam->qIdRefOffset++;
        }
    }

}




/**
* <B> Function: MCAPP_FWeakUpdateMotorParameters(MCAPP_UPDATE_PARAMS_T * )  </B>
*
* @brief Function to update motor parameters during field weakening
*
* @param Pointer to the data structure containing Field Weakening Control
*        parameters.
* @return   none.
* @example
* <CODE> MCAPP_FWeakUpdateMotorParameters(&UpdateParams); </CODE>
*
*/

static void MCAPP_FWeakUpdateMotorParameters(MCAPP_UPDATE_PARAMS_T *pUpdateParams)
{    
    const MCAPP_CONTROL_T *pCtrlParam = pUpdateParams->pCtrlParam;
    MCAPP_ESTIMATOR_MRAS_T *pEstimMRAS = pUpdateParams->pEstimMRAS;

    int32_t InvLoRatio;
    /* LoRatio  = Lo/LoN   
     * InvLoRatio  = LoN/Lo */
    
    /* InvLoRatio = coef1 * x + coef2 */         
    InvLoRatio = (int32_t)(__builtin_mulss(pUpdateParams->paramUpdateCoef1,
                   pCtrlParam->qIdRef) >> pUpdateParams->paramUpdateCoef1Qvalue)
                                    + (int32_t)(pUpdateParams->paramUpdateCoef2);
    if(InvLoRatio < INVLORATIO_MIN )
    {
        pUpdateParams->InvLoRatio = INVLORATIO_MIN ;
    }
    else if(InvLoRatio > INVLORATIO_MAX)
    {
        pUpdateParams->InvLoRatio = INVLORATIO_MAX;
    }
    else{
        pUpdateParams->InvLoRatio = (int16_t)InvLoRatio;
    }
    /* LoRatioByTwo  = 0.5 / InvLoRatio */
    pUpdateParams->LoRatioByTwo  = (int16_t)__builtin_divf( Q14_1 ,
                                                    pUpdateParams->InvLoRatio);
     
    
    
    /* Parameter update for MRAS estimator*/
    pEstimMRAS->Ls      = (int16_t)(__builtin_mulss(pUpdateParams->LoRatioByTwo,
                                                  pUpdateParams->Nom_Ls) >> 14);
    pEstimMRAS->sigmaLs = (int16_t)(__builtin_mulss(pUpdateParams->LoRatioByTwo, 
                                             pUpdateParams->Nom_sigmaLs) >> 14); 
    pEstimMRAS->InvTr   = (int16_t)(__builtin_mulss(pUpdateParams->InvLoRatio, 
                                               pUpdateParams->Nom_InvTr) >> 15);      
}