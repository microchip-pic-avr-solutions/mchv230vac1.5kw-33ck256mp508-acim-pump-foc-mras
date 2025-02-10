// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file mc1_init.c
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

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "mc1_init.h"
#include "mc1_calc_params.h"
#include "mc_app_types.h"

#include "board_service.h"
#include "application/application.h"
#include "application/application_types.h"

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="STATIC FUNCTIONS ">

static void MCAPP_MC1ControlSchemeConfig(MC1APP_DATA_T *);
static void MCAPP_MC1LoadConfig(MC1APP_DATA_T *);
static void MCAPP_MC1FeedbackConfig(MC1APP_DATA_T *);
static void MCAPP_MC1LoadStartTransition(MCAPP_CONTROL_SCHEME_T *, 
                                            MCAPP_LOAD_T *);
static void MCAPP_MC1LoadStopTransition(MCAPP_CONTROL_SCHEME_T *, 
                                            MCAPP_LOAD_T *);
static void MCAPP_MC1OutputConfig(MC1APP_DATA_T *);

// </editor-fold>

/**
* <B> Function: MCAPP_MC1ParamsInit (MC1APP_DATA_T *)  </B>
*
* @brief Function to reset variables used for current offset measurement.
*
* @param Pointer to the Application data structure required for 
* controlling motor 1.
* @return none.
* @example
* <CODE> MCAPP_MC1ParamsInit(&mc1); </CODE>
*
*/
void MCAPP_MC1ParamsInit(MC1APP_DATA_T *pMCData)
{    
    /* Reset all variables in the data structure to '0' */
    memset(pMCData,0,sizeof(MC1APP_DATA_T));

    pMCData->pControlScheme = &pMCData->controlScheme;
    pMCData->pMotorInputs = &pMCData->motorInputs;
    pMCData->pLoad = &pMCData->load;
    pMCData->pMotor = &pMCData->motor;
    pMCData->pPWMDuty = &pMCData->PWMDuty;
    
    /* Configure Feedbacks */
    MCAPP_MC1FeedbackConfig(pMCData);
    
    /* Configure Control Scheme */
    MCAPP_MC1ControlSchemeConfig(pMCData);
    

    /* Configure Load */
    MCAPP_MC1LoadConfig(pMCData);
    
    /* Configure Outputs */
    MCAPP_MC1OutputConfig(pMCData);

    /* Set motor control state as 'MTR_INIT' */
    pMCData->appState = MCAPP_INIT;
}

void MCAPP_MC1FeedbackConfig(MC1APP_DATA_T *pMCData)
{
    pMCData->HAL_MotorInputsRead = HAL_MC1MotorInputsRead;
    
    pMCData->motorInputs.measureVdc.dcMinRun = 
                        NORM_VALUE(MC1_MOTOR_MIN_DC_VOLT, MC1_PEAK_VOLTAGE);
    
    pMCData->motorInputs.measureVdc.dcMaxStop = 
                        NORM_VALUE(MC1_MOTOR_MIN_DC_VOLT, MC1_PEAK_VOLTAGE);

}

void MCAPP_MC1ControlSchemeConfig(MC1APP_DATA_T *pMCData)
{
    MCAPP_CONTROL_SCHEME_T *pControlScheme;
    MCAPP_MEASURE_T *pMotorInputs;
    MCAPP_MOTOR_T *pMotor;
    
    
    pControlScheme = pMCData->pControlScheme;
    pMotorInputs = pMCData->pMotorInputs;
    pMotor = pMCData->pMotor;
    
    /* Configure Inputs */  
    pControlScheme->pIa = &pMotorInputs->measureCurrent.Ia;
    pControlScheme->pIb = &pMotorInputs->measureCurrent.Ib;
    pControlScheme->pVdc = &pMotorInputs->measureVdc.value;  
    pControlScheme->pMotor = pMCData->pMotor;
    
    /* Initialize IMotor parameters */
    pMotor->polePairs = POLEPAIRS;
    pMotor->qLs                  = NORM_LS; 
    pMotor->qLsScale             = NORM_LS_QVALUE;

    pMotor->qNominalSpeed   = NORM_VALUE(NOMINAL_SPEED_RPM, MC1_PEAK_SPEED_RPM);
    pMotor->qMaxSpeed       = NORM_VALUE(MAXIMUM_SPEED_RPM, MC1_PEAK_SPEED_RPM);
    pMotor->qMaxOLSpeed     = NORM_VALUE(END_SPEED_RPM, MC1_PEAK_SPEED_RPM);
    pMotor->qMinSpeed       = NORM_VALUE(MINIMUM_SPEED_RPM, MC1_PEAK_SPEED_RPM);

    pMotor->qRatedCurrent = NORM_VALUE(RATED_CURRENT, MC1_PEAK_CURRENT);
    pMotor->qNominalMagCurrent = NORM_VALUE(NOMINAL_MAGNITIZING_CURRENT_PEAK,
                                                            MC1_PEAK_CURRENT) ;
    pMotor->qMinMagCurrent = NORM_VALUE(MIN_MAGNITIZING_CURRENT_PEAK,
                                                            MC1_PEAK_CURRENT) ;

    /* Initialize FOC control parameters */
#ifdef  OPEN_LOOP_FUNCTIONING
    pControlScheme->ctrlParam.openLoop = 1;
#else
    pControlScheme->ctrlParam.openLoop = 0;
#endif            
    
#ifdef  FD_WEAK_TYPE_VOLTAGE_FB
    pControlScheme->ctrlParam.fluxWeakType = 2;
#else
    pControlScheme->ctrlParam.fluxWeakType = 1;
#endif  

    pControlScheme->ctrlParam.speedRampIncLimit = RAMP_UP_TIME_MULTIPLIER;
    pControlScheme->ctrlParam.speedRampDecLimit = RAMP_DN_TIME_MULTIPLIER;

    pControlScheme->ctrlParam.qTargetVelocity = pMotor->qMinSpeed;
    
    pControlScheme->ctrlParam.CLSpeedRampRate = SPEED_RAMP_RATE_COUNT;
    
    pControlScheme->ctrlParam.normDeltaT = NORM_DELTA_T;
    
     
    /* Initialize PI controller used for D axis current control */
    pControlScheme->piId.kp = D_CURRCNTR_PTERM;
    pControlScheme->piId.nkp = D_CURRCNTR_PTERM_SCALE;
    pControlScheme->piId.ki = D_CURRCNTR_ITERM;
    pControlScheme->piId.nki = D_CURRCNTR_ITERM_SCALE;
    pControlScheme->piId.outMax = D_CURRCNTR_OUTMAX;
    pControlScheme->piId.outMin = (-pControlScheme->piId.outMax);
    pControlScheme->piId.kc =  Q15(0.999);

    /* Initialize PI controller used for Q axis current control */
    pControlScheme->piIq.kp = Q_CURRCNTR_PTERM;
    pControlScheme->piIq.nkp = Q_CURRCNTR_PTERM_SCALE;
    pControlScheme->piIq.ki = Q_CURRCNTR_ITERM;
    pControlScheme->piIq.nki = Q_CURRCNTR_ITERM_SCALE;
    pControlScheme->piIq.outMax = Q_CURRCNTR_OUTMAX;
    pControlScheme->piIq.outMin = (-pControlScheme->piIq.outMax);
    pControlScheme->piIq.kc =  Q15(0.999);

    /* Initialize PI controller used for speed control */
    pControlScheme->piSpeed.kp = SPEEDCNTR_PTERM;
    pControlScheme->piSpeed.ki = SPEEDCNTR_ITERM;
    pControlScheme->piSpeed.nkp = SPEEDCNTR_PTERM_SCALE;
    pControlScheme->piSpeed.nki = SPEEDCNTR_ITERM_SCALE;
    pControlScheme->piSpeed.outMax = SPEEDCNTR_OUTMAX;
    pControlScheme->piSpeed.outMin = 0;
    pControlScheme->piSpeed.kc =  Q15(0.999);

    
    /* Initialize MRAS Estimator Parameters*/
    pControlScheme->estimMRAS.pCtrlParam = &pControlScheme->ctrlParam;
    pControlScheme->estimMRAS.pVdq       = &pControlScheme->vdq;
    pControlScheme->estimMRAS.pMotor     = pMCData->pMotor;
    pControlScheme->estimMRAS.pIdq       = &pControlScheme->idq;
    pControlScheme->estimMRAS.Ls         =  NORM_LS;
    pControlScheme->estimMRAS.LsScale    = NORM_LS_QVALUE;
    pControlScheme->estimMRAS.sigmaLs    = SIGMA_LS;
    pControlScheme->estimMRAS.sigmaLsScale = SIGMA_LS_QVALUE;
    pControlScheme->estimMRAS.qImrEstimFilterK = KFILTER_IMRESTIM;
    pControlScheme->estimMRAS.qDeltaT       = NORM_DELTA_T;
    pControlScheme->estimMRAS.InvTr         = NORM_INVTR;
    pControlScheme->estimMRAS.qOmegaFiltConst = KFILTER_OMEGAR;
    pControlScheme->estimMRAS.MRAS_PISTATE.kp      = MRAS_PTERM;
    pControlScheme->estimMRAS.MRAS_PISTATE.ki      = MRAS_ITERM;
    pControlScheme->estimMRAS.MRAS_PISTATE.kc      = Q15(0.99999);
    pControlScheme->estimMRAS.MRAS_PISTATE.nkp     = MRAS_PTERM_SCALE;
    pControlScheme->estimMRAS.MRAS_PISTATE.nki     = 0;
    pControlScheme->estimMRAS.MRAS_PISTATE.outMax  = DELTA_OMEGA_MAX;
    pControlScheme->estimMRAS.MRAS_PISTATE.outMin  = -DELTA_OMEGA_MAX;
    pControlScheme->estimMRAS.LPF_MRAS_omegaSyn.X_Coeff = LPF_MRAS_OMEGASYN_XCOEF;
    pControlScheme->estimMRAS.LPF_MRAS_omegaSyn.Y_Coeff = LPF_MRAS_OMEGASYN_YCOEF;
    
    /* Initialize field weakening controller 2*/ 
    pControlScheme->idRefGen.pFWeakType2 = &pControlScheme->idRefGen.fWeakType2;
    pControlScheme->idRefGen.fWeakType2.pCtrlParam = &pControlScheme->ctrlParam;
    pControlScheme->idRefGen.fWeakType2.pMotor = pMCData->pMotor;
    pControlScheme->idRefGen.fWeakType2.pVdq = &pControlScheme->vdq;
    pControlScheme->idRefGen.fWeakType2.voltageMagRef = FD_WEAK_VOLTAGE_REF;
    pControlScheme->idRefGen.fWeakType2.FWeakPI.kp = FD_WEAK_PI_KP;
    pControlScheme->idRefGen.fWeakType2.FWeakPI.ki = FD_WEAK_PI_KI; 
    pControlScheme->idRefGen.fWeakType2.FWeakPI.kc = Q15(0.999);
    pControlScheme->idRefGen.fWeakType2.FWeakPI.nkp = FD_WEAK_PI_KPSCALE;
    pControlScheme->idRefGen.fWeakType2.FWeakPI.nki = 0;
    pControlScheme->idRefGen.fWeakType2.IdRefFiltConst = FD_WEAK_IDREF_FILT_CONST;
    pControlScheme->idRefGen.fWeakType2.IdRefMin = pMotor->qMinMagCurrent;
    

    /* Initialize field weakening controller 2 */ 
    pControlScheme->idRefGen.pFWeakType1 = &pControlScheme->idRefGen.fWeakType1;
    pControlScheme->idRefGen.fWeakType1.pCtrlParam = &pControlScheme->ctrlParam;
    pControlScheme->idRefGen.fWeakType1.pMotor = pMCData->pMotor;
    pControlScheme->idRefGen.fWeakType1.FdweakStartSpeed = FD_WEAK_START_SPEED;
    pControlScheme->idRefGen.fWeakType1.IdRefMin = pMotor->qMinMagCurrent;
    
    /* Initialize parameter update function in field weakening */ 
    pControlScheme->idRefGen.pUpdateParms = &pControlScheme->idRefGen.UpdateParms;
    pControlScheme->idRefGen.UpdateParms.pCtrlParam = &pControlScheme->ctrlParam;
    pControlScheme->idRefGen.UpdateParms.pMotor =     pMCData->pMotor;
    pControlScheme->idRefGen.UpdateParms.pEstimMRAS = &pControlScheme->estimMRAS;        
    pControlScheme->idRefGen.UpdateParms.Nom_InvTr = NORM_INVTR;
    pControlScheme->idRefGen.UpdateParms.Nom_Ls  = NORM_LS;
    pControlScheme->idRefGen.UpdateParms.Nom_sigmaLs = SIGMA_LS;
    pControlScheme->idRefGen.UpdateParms.paramUpdateCoef1 = PARAMETER_UPDATE_COEF1;
    pControlScheme->idRefGen.UpdateParms.paramUpdateCoef2 = PARAMETER_UPDATE_COEF2;
    pControlScheme->idRefGen.UpdateParms.paramUpdateCoef1Qvalue 
                                                 = PARAMETER_UPDATE_COEF1QVALUE;
    
    /* Output Initializations */
    pControlScheme->pwmPeriod   = MC1_LOOPTIME_TCY;
    pControlScheme->pPWMDuty    = pMCData->pPWMDuty;
    
    /* Initialize application structure */
    pMCData->MCAPP_ControlSchemeInit = MCAPP_FOCInit;
    pMCData->MCAPP_ControlStateMachine = MCAPP_FOCStateMachine;
    
    pMCData->MCAPP_InputsInit = MCAPP_MeasureCurrentInit;
    pMCData->MCAPP_MeasureOffset = MCAPP_MeasureCurrentOffset;
    pMCData->MCAPP_GetProcessedInputs = MCAPP_MeasureCurrentCalibrate;
    pMCData->MCAPP_IsOffsetMeasurementComplete = 
                                       MCAPP_MeasureCurrentOffsetStatus;
    
}

/**
* <B> Function: MCAPP_MC1LoadConfig (MC1APP_DATA_T *)  </B>
*
* @brief Function to reset variables used for current offset measurement.
*
* @param Pointer to the FOC data structure required for controlling motor 1.
* @return none.
* @example
* <CODE> MCAPP_MC1LoadConfig(&mcData); </CODE>
*
*/

void MCAPP_MC1LoadConfig(MC1APP_DATA_T *pMCData)
{

    
	pMCData->MCAPP_LoadInit = MCAPP_ApplicationInit;
    pMCData->MCAPP_LoadStateMachine = MCAPP_ApplicationStateMachine;
    pMCData->MCAPP_IsLoadReadyToStart = MCAPP_IsApplicationReadyToStart;
    pMCData->MCAPP_IsLoadReadyToStop = MCAPP_IsApplicationReadyToStop;
    
    pMCData->MCAPP_LoadStartTransition = MCAPP_MC1LoadStartTransition;
    pMCData->MCAPP_LoadStopTransition = MCAPP_MC1LoadStopTransition;
}


void MCAPP_MC1LoadStartTransition(MCAPP_CONTROL_SCHEME_T *pControlScheme, 
                                    MCAPP_LOAD_T *pLoad)
{
    pControlScheme->focState = FOC_OPEN_LOOP; 
    pLoad->state = APPLICATION_RUN;
}

void MCAPP_MC1LoadStopTransition(MCAPP_CONTROL_SCHEME_T *pControlScheme, 
                                    MCAPP_LOAD_T *pLoad)
{
    pLoad->state = APPLICATION_STOP;
}

void MCAPP_MC1OutputConfig(MC1APP_DATA_T *pMCData)
{
    pMCData->HAL_PWMSetDutyCycles = HAL_MC1PWMSetDutyCycles;
    pMCData->HAL_PWMEnableOutputs = HAL_MC1PWMEnableOutputs;
    pMCData->HAL_PWMDisableOutputs = HAL_MC1PWMDisableOutputs;
    pMCData->MCAPP_HALSetVoltageVector = HAL_MC1SetVoltageVector;
}