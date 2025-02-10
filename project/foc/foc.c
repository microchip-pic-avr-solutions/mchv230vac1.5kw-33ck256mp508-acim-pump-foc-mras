// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file foc.c
 *
 * @brief This module implements Field Oriented Control(FOC).
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

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include <stdbool.h>

#include <libq.h>

#include "foc.h"
#include "general.h"
#include "hal/port_config.h"

#include "../mc1_user_params.h"

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Definitions ">
#define Q14_SQRT_3  28377 /* sqrt(3)  */
// </editor-fold>
// <editor-fold defaultstate="collapsed" desc="STATIC FUNCTIONS ">

static void MCAPP_FOCFeedbackPath(MCAPP_FOC_T *);
static void MCAPP_FOCForwardPath(MCAPP_FOC_T *);
static void MCAPP_VbyFForwardPath(MCAPP_FOC_T *); 
static void MCAPP_VoltageBaseChange(MC_ABC_T *, MC_ABC_T *);
static void MCAPP_DCLinkVoltageCompensation(MC_ABC_T *, MC_ABC_T *, int16_t* );


// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="GLOBAL FUNCTIONS ">

void MCAPP_FOCStateMachine(MCAPP_FOC_T *);
void MCAPP_FOCInit(MCAPP_FOC_T *);

// </editor-fold>

/**
* <B> Function: void MCAPP_FOCInit(MCAPP_FOC_T *)  </B>
*
* @brief Executes FOC Parameters Initialization.
*
* @param Pointer to the data structure containing FOC parameters.
* @return none.
* @example
* <CODE> MCAPP_FOCInit(&mc); </CODE>
*
*/

void MCAPP_FOCInit(MCAPP_FOC_T *pFOC)
{
    MCAPP_CONTROL_T *pCtrlParam = &pFOC->ctrlParam;
    
    MCAPP_ControllerPIInit(&pFOC->piId);
    MCAPP_ControllerPIInit(&pFOC->piIq);
    MCAPP_ControllerPIInit(&pFOC->piSpeed);
    
    MCAPP_EstimatorMRASInit(&pFOC->estimMRAS); 
    MCAPP_IdRefGenerationInit(&pFOC->idRefGen);
    
    pCtrlParam->lockTime = 0;
    pCtrlParam->speedRampSkipCnt = 0;
    pCtrlParam->OLTheta = 0;
    pCtrlParam->qVelRef = 0;
    pCtrlParam->OLThetaSum = 0;
    pCtrlParam->qIdRef = 0;
    pCtrlParam->qIqRef = 0;
    
    pFOC->faultStatus = 0;
    
    pFOC->pPWMDuty->dutycycle3 = 0;
    pFOC->pPWMDuty->dutycycle2 = 0;
    pFOC->pPWMDuty->dutycycle1 = 0;
}

/**
* <B> Function: void MCAPP_FOCControlLoop(MCAPP_FOC_T *)  </B>
*
* @brief Executes Speed and Current Control Loops and performs actions
*        associated with control state transition required for FOC.
*
* @param Pointer to the data structure containing FOC parameters.
* @return none.
* @example
* <CODE> MCAPP_FOCControlLoop(&mc); </CODE>
*
*/
void MCAPP_FOCStateMachine(MCAPP_FOC_T *pFOC)
{


    MCAPP_CONTROL_T *pCtrlParam = &pFOC->ctrlParam;


    
    switch (pFOC->focState)
    {
        case FOC_INIT:
            MCAPP_FOCInit(pFOC); 
            break;
        
        case FOC_OPEN_LOOP:
            MCAPP_FOCFeedbackPath(pFOC); 
             
            pCtrlParam->qDiff = pCtrlParam->qVelRef - 
                                          (int16_t)pCtrlParam->qTargetVelocity;

            if(pCtrlParam->qDiff < 0)
            {
                if(pCtrlParam->speedRampSkipCnt >= 
                                pCtrlParam->speedRampIncLimit)
                {
                    pCtrlParam->speedRampSkipCnt = 0;
                    pCtrlParam->qVelRef = pCtrlParam->qVelRef + 
                                                    pCtrlParam->CLSpeedRampRate;
                }
            }
            else if(pCtrlParam->qDiff > 0)
            {
                if(pCtrlParam->speedRampSkipCnt >= 
                                pCtrlParam->speedRampDecLimit)
                {
                    pCtrlParam->speedRampSkipCnt = 0;
                    pCtrlParam->qVelRef = pCtrlParam->qVelRef - 
                                    pCtrlParam->CLSpeedRampRate;
                }
            }
            else
            {
                pCtrlParam->speedRampSkipCnt = 0;
            }
            pCtrlParam->speedRampSkipCnt++;


            /* Calculate open loop theta */
            pCtrlParam->OLThetaSum += __builtin_mulss(pCtrlParam->qVelRef, 
                                                        pCtrlParam->normDeltaT);           
            pCtrlParam->OLTheta = (int16_t)(pCtrlParam->OLThetaSum >> 15);
            
            pFOC->estimInterface.qTheta  = pCtrlParam->OLTheta;

            pCtrlParam->qIdRef = pFOC->pMotor->qNominalMagCurrent; 
            
            
            if(pCtrlParam->qVelRef > pFOC->pMotor->qMaxOLSpeed)
            {
                if(pCtrlParam->openLoop == 0)
                {
                    
                    pFOC->estimMRAS.qRhoStateVar = pCtrlParam->OLThetaSum;
                    /* Reset voltage PI controllers*/
                    MCAPP_ControllerPIReset(&pFOC->piIq, pFOC->vdq.q);
                    MCAPP_ControllerPIReset(&pFOC->piId, pFOC->vdq.d);
                    /* Id reference offset value */
                    pCtrlParam->qIdRefOffset = pCtrlParam->qIdRef - pFOC->idq.d;
                    /* Reset speed PI controller */
                    MCAPP_ControllerPIReset(&pFOC->piSpeed, pFOC->idq.q);
                    
                    pFOC->focState = FOC_CLOSE_LOOP;
                }
            }

            
            MCAPP_VbyFForwardPath(pFOC);
            break;
            
        case FOC_CLOSE_LOOP:

            MCAPP_FOCFeedbackPath(pFOC);
            
            pFOC->estimInterface.qTheta  = pFOC->estimMRAS.qRho ;                            

            /* Closed Loop Speed Ramp */
            pCtrlParam->qDiff = pCtrlParam->qVelRef - 
                                        (int16_t)pCtrlParam->qTargetVelocity;

            if(pCtrlParam->qDiff < 0)
            {
                if(pCtrlParam->speedRampSkipCnt >= 
                                pCtrlParam->speedRampIncLimit)
                {
                    pCtrlParam->speedRampSkipCnt = 0;
                    pCtrlParam->qVelRef = pCtrlParam->qVelRef + 
                                                    pCtrlParam->CLSpeedRampRate;
                }
            }
            else if(pCtrlParam->qDiff > 0)
            {
                if(pCtrlParam->speedRampSkipCnt >= 
                                pCtrlParam->speedRampDecLimit)
                {
                    pCtrlParam->speedRampSkipCnt = 0;
                    pCtrlParam->qVelRef = pCtrlParam->qVelRef - 
                                    pCtrlParam->CLSpeedRampRate;
                }
            }
            else
            {
                pCtrlParam->speedRampSkipCnt = 0;
            }
            pCtrlParam->speedRampSkipCnt++;

            /* Execute Outer Speed Loop - Generate Iq Reference Generation */

            pFOC->estimInterface.qVelEstim =  pFOC->estimMRAS.omegaRotorFil;

            
            MCAPP_ControllerPIUpdate(pCtrlParam->qVelRef, pFOC->estimInterface.qVelEstim, 
            &pFOC->piSpeed, MCAPP_SAT_NONE, &pCtrlParam->qIqRef,
            pCtrlParam->qVelRef);
                    
            /* Generate Id Reference : Flux Weakening and update estimator parameters */
            MCAPP_IdRefGeneration(&pFOC->idRefGen);
   
            if(pCtrlParam->fluxWeakType == 1){
                pCtrlParam->qIdRef = pFOC->idRefGen.fWeakType1.IdRef;
            }
            else{
                pCtrlParam->qIdRef = pFOC->idRefGen.fWeakType2.IdRef;
            }
           

            MCAPP_FOCForwardPath(pFOC);
            break;

        case FOC_FAULT:
                    
            break;
        
        default:
            pFOC->focState = FOC_FAULT;
            break;

    } /* End Of switch - case */
}

/**
* <B> Function: void MCAPP_FOCFeedbackPath (MCAPP_FOC_T *)  </B>
*
* @brief Function executing Field Oriented Control(FOC).
*
* @param Pointer to the data structure containing FOC parameters.
* @return none.
* @example
* <CODE> MCAPP_FOCFeedbackPath(&mc); </CODE>
*
*/
static void MCAPP_FOCFeedbackPath(MCAPP_FOC_T *pFOC)
{
    pFOC->iabc.a = *(pFOC->pIa);
    pFOC->iabc.b = *(pFOC->pIb);
    pFOC->iabc.c = -pFOC->iabc.a - pFOC->iabc.b;
    
    
    /* Perform Clark & Park transforms to generate d axis and q axis currents */
    MC_TransformClarke_Assembly(&pFOC->iabc, &pFOC->ialphabeta);
    MC_TransformPark_Assembly(&pFOC->ialphabeta, 
                                &pFOC->sincosTheta, &pFOC->idq);

    /* Estimate Speed and rotor position */  
    MCAPP_EstimatorMRAS(&pFOC->estimMRAS); 
}

/**
* <B> Function: void MCAPP_FOCForwardPath(MCAPP_FOC_T *)  </B>
*
* @brief Executes Current Control Loops required for FOC.
*
* @param Pointer to the data structure containing FOC parameters.
* @return none.
* @example
* <CODE> MCAPP_FOCForwardPath(&mc); </CODE>
*
*/

static void MCAPP_FOCForwardPath(MCAPP_FOC_T *pFOC)
{
    int16_t vqSquaredLimit, vdSquared;
    
    /** Execute inner current control loops */
    /* Execute PI Control of D axis. */
    MCAPP_ControllerPIUpdate(pFOC->ctrlParam.qIdRef,  pFOC->idq.d, 
            &pFOC->piId, MCAPP_SAT_NONE, &pFOC->vdq.d,
            pFOC->ctrlParam.qIdRef);

    /* Generate Q axis current reference based on available voltage and D axis
       voltage */
    vdSquared  = (int16_t)(__builtin_mulss(pFOC->vdq.d, pFOC->vdq.d)>>15);
    vqSquaredLimit = MAX_VOLTAGE_SQUARE - vdSquared;  

    pFOC->piIq.outMax = _Q15sqrt(vqSquaredLimit);
    pFOC->piIq.outMin = -(pFOC->piIq.outMax); 
    
    /* Execute PI Control of Q axis. */ 
    MCAPP_ControllerPIUpdate(pFOC->ctrlParam.qIqRef,  pFOC->idq.q, 
            &pFOC->piIq, MCAPP_SAT_NONE, &pFOC->vdq.q,
            pFOC->ctrlParam.qIqRef);
    
    
    /* Calculate sin and cos of theta (angle) */
    MC_CalculateSineCosine_Assembly_Ram(pFOC->estimInterface.qTheta, 
                                            &pFOC->sincosTheta);

    /* Perform inverse Clarke and Park transforms and generate phase voltages.*/
    MC_TransformParkInverse_Assembly(&pFOC->vdq, &pFOC->sincosTheta, 
                                                        &pFOC->valphabeta);

    /* Calculate Vr1,Vr2,Vr3 from qValpha, qVbeta */
    MC_TransformClarkeInverseSwappedInput_Assembly(&pFOC->valphabeta, 
                                                        &pFOC->vabc);
    
    /* Scaling vabc to the base of MC_CalculateSpaceVectorPhaseShifted_Assembly */
    MCAPP_VoltageBaseChange(&pFOC->vabc, &pFOC->vabcScaled);
    
        /* DC Link voltage compensation */
    MCAPP_DCLinkVoltageCompensation(&pFOC->vabcScaled, &pFOC->vabcCompDC, pFOC->pVdc);

    /* Execute space vector modulation and generate PWM duty cycles */
    MC_CalculateSpaceVectorPhaseShifted_Assembly(&pFOC->vabcCompDC, pFOC->pwmPeriod,
                                                    pFOC->pPWMDuty);
}


/**
* <B> Function: void MCAPP_VbyFForwardPath(MCAPP_FOC_T *)  </B>
*
* @brief Executes open loop control required for Voltage/Frequency control.
*
* @param Pointer to the data structure containing FOC parameters.
* @return none.
* @example
* <CODE> MCAPP_VbyFForwardPath(&mc); </CODE>
*
*/
static void MCAPP_VbyFForwardPath(MCAPP_FOC_T *pFOC)
{
    int16_t vqRefOpenLoop;
 
    /* Multiplying the V/F constant with Speed ref to get required voltage to apply */
    vqRefOpenLoop = (int16_t) (__builtin_mulss(VBYF_CONSTANT,pFOC->ctrlParam.qVelRef) >> VBYF_CONSTANT_SHIFT);

    /* Limit the vqRefOpenLoop to Open Loop Maximum Modulation index */
    if(vqRefOpenLoop >= VMAX_VBYF_CONTROL)
    {
       vqRefOpenLoop =  VMAX_VBYF_CONTROL;
    } 
    
    pFOC->vdq.q = vqRefOpenLoop;
    pFOC->vdq.d = 0;
    
    /* Calculate sin and cos of theta (angle) */
    MC_CalculateSineCosine_Assembly_Ram(pFOC->estimInterface.qTheta, 
                                                        &pFOC->sincosTheta);

    /* Perform inverse Clarke and Park transforms and generate phase voltages.*/
    MC_TransformParkInverse_Assembly(&pFOC->vdq, &pFOC->sincosTheta, 
                                                        &pFOC->valphabeta);

    /* Calculate Vr1,Vr2,Vr3 from qValpha, qVbeta */
    MC_TransformClarkeInverseSwappedInput_Assembly(&pFOC->valphabeta, 
                                                        &pFOC->vabc);

    /* Scaling vabc to the base of MC_CalculateSpaceVectorPhaseShifted_Assembly */
    MCAPP_VoltageBaseChange(&pFOC->vabc, &pFOC->vabcScaled);
    
    /* DC Link voltage compensation */
    MCAPP_DCLinkVoltageCompensation(&pFOC->vabcScaled, &pFOC->vabcCompDC, pFOC->pVdc);
    
    /* Execute space vector modulation and generate PWM duty cycles */
    MC_CalculateSpaceVectorPhaseShifted_Assembly(&pFOC->vabcScaled, pFOC->pwmPeriod,
                                                    pFOC->pPWMDuty);
}

/**
* <B> Function: void MCAPP_VoltageBaseChange(MC_ABC_T *, MC_ABC_T *)  </B>
*
*/
static void MCAPP_VoltageBaseChange(MC_ABC_T *pabcIn, MC_ABC_T *pvabcOut)
{
    /* Multiply by Root(3) */
    pvabcOut->a = (int16_t) (__builtin_mulss(pabcIn->a, Q14_SQRT_3) >> 14);
    pvabcOut->b = (int16_t) (__builtin_mulss(pabcIn->b, Q14_SQRT_3) >> 14);
    pvabcOut->c = (int16_t) (__builtin_mulss(pabcIn->c, Q14_SQRT_3) >> 14);
}

/**
* <B> Function: void MCAPP_VoltageBaseChange(MC_ABC_T *, MC_ABC_T *)  </B>
 *  @brief Executes DC voltage compensation
 * 
*/
static void MCAPP_DCLinkVoltageCompensation(MC_ABC_T *pvabc, MC_ABC_T *pdabc, int16_t *pvdc)
{
    int16_t rVdc, rVdc_scale=0;
    const int16_t vdc = *pvdc ;
    if (vdc > DC_BUS_COMP)
    {
        rVdc = __builtin_divf(DC_BUS_COMP, vdc);
        rVdc_scale = 0;
    }
    else if (vdc > (DC_BUS_COMP>>1))
    {
        rVdc = __builtin_divf((DC_BUS_COMP>>1), vdc);
        rVdc_scale = 1;
    }
    else
    {
        rVdc = 0;
        rVdc_scale = 0;
    }
    pdabc->a = (int16_t) (__builtin_mulss(pvabc->a, rVdc) >> (15-rVdc_scale));
    pdabc->b = (int16_t) (__builtin_mulss(pvabc->b, rVdc) >> (15-rVdc_scale));
    pdabc->c = (int16_t) (__builtin_mulss(pvabc->c, rVdc) >> (15-rVdc_scale));
}
