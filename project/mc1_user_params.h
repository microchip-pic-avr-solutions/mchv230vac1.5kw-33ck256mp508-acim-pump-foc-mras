// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file mc1_user_params.h
 *
 * @brief This file has definitions to be configured by the user for spinning
 * 		  motor 1 using field oriented control.
 *
 * Component: APPLICATION (motor control 1 - mc1)
 * Motor : ----------Induction Motor
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

#ifndef __MC1_USER_PARAMS_H
#define __MC1_USER_PARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>

#include "general.h"

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="DEFINITIONS/MACROS ">

/** Define macros for operational Modes */
    
/* Define OPEN_LOOP_FUNCTIONING for Open loop continuous functioning, 
 * undefine OPEN_LOOP_FUNCTIONING for closed loop functioning  */
#undef OPEN_LOOP_FUNCTIONING 
    
/* Select Field weakening method 
 * Define the FD_WEAK_TYPE_VOLTAGE_FB macro for voltage feedback based Field 
 * weakening control. Undefine the macro for conventional inverse speed 
 * based Field weakening control
 * */    
#define FD_WEAK_TYPE_VOLTAGE_FB

    
/** Board Parameters */
#define     MC1_PEAK_VOLTAGE            453.3  /* Peak measurement voltage of the board */
#define     MC1_PEAK_CURRENT            22     /* Peak measurement current of the board */
    
/** Motor Parameters */

/* Define Motor */    
#define PUMP_MOTOR_1  
  
/* Enter the minimum DC link voltage(V) required to run the motor*/    
#define   MC1_MOTOR_MIN_DC_VOLT     250 
    
/** The following values are given in the .xlsx file. */   
#ifdef PUMP_MOTOR_1
    #define POLEPAIRS           1  /* Motor's number of pole pairs */
    #define NOMINAL_SPEED_RPM   3450 /* Nominal speed of the motor in RPM */
    #define MAXIMUM_SPEED_RPM   3600 /* Maximum speed of the motor in RPM */
    #define MINIMUM_SPEED_RPM   500 /* Minimum speed of the motor in RPM*/
    /*Motor Rated Line - Line RMS Voltage*/
    #define NOMINAL_VOLTAGE_L_L  (float)230
    /* Motor Rated Phase Current RMS in Amps */
    #define NOMINAL_CURRENT_PHASE_RMS (float) 7.8
    /* Motor Rated Magnetization Current RMS in Amps */
    #define NOMINAL_MAGNITIZING_CURRENT_RMS (float) 2
    /* Motor Rated Torque Component of Current RMS in Amps */
    #define NOMINAL_TORQUE_COMPONENT_CURRENT_RMS (float) 6.6

    /* Base values entered in .xlsx file*/
                                      /* Base Current = MC1_PEAK_CURRENT */
    #define MC1_BASE_VOLTAGE    325  /* Vdc Base voltage = Rated voltage*1.414*/
    #define MC1_PEAK_SPEED_RPM  2.5*NOMINAL_SPEED_RPM   /* Base Speed in RPM */

    /*Maximum utilizable Voltage in V/F Control (open loop Control)*/
    #define VMAX_VBYF_CONTROL   (int16_t)((float)0.9*18918) /* 0.9* Vdclink/root3, 32767/sqrt(3) = 18918*/
    /*Maximum utilizable Voltage Limit in closed loop control*/
    #define VMAX_CLOSEDLOOP_CONTROL  (int16_t)((float)0.83*18918) /* 0.85* Vdclink/root3 */ 
    /* Motor Rated Torque Component of Current Peak in Amps */
    #define NOMINAL_TORQUE_COMPONENT_CURRENT_PEAK (float) (NOMINAL_TORQUE_COMPONENT_CURRENT_RMS * 1.414)

    /* The following values are given in the xls attached file */
    /* Open loop VbyF control parameters */
    #define	VBYF_CONSTANT       23668
    #define	VBYF_CONSTANT_SHIFT	14

    /* MRAS Estimator Parameters */
	#define	SIGMA_LS        18699
	#define	SIGMA_LS_QVALUE      15
	#define	NORM_LS         19213
	#define	NORM_LS_QVALUE	11
    #define	NORM_DELTA_T	471
    #define	KFILTER_IMRESTIM    16
    #define	NORM_INVTR      352


    /* PI controllers tuning values - */     
    /* D Control Loop Coefficients */
    #define	Q_CURRCNTR_PTERM        11502
    #define	Q_CURRCNTR_PTERM_SCALE  2
    #define	Q_CURRCNTR_ITERM        518
    #define	Q_CURRCNTR_ITERM_SCALE  0
    #define Q_CURRCNTR_OUTMAX       VMAX_CLOSEDLOOP_CONTROL

    /* Q Control Loop Coefficients */
    #define D_CURRCNTR_PTERM        Q_CURRCNTR_PTERM
    #define D_CURRCNTR_PTERM_SCALE  Q_CURRCNTR_PTERM_SCALE
    #define D_CURRCNTR_ITERM        Q_CURRCNTR_ITERM
    #define D_CURRCNTR_ITERM_SCALE  Q_CURRCNTR_ITERM_SCALE
    #define D_CURRCNTR_OUTMAX       VMAX_CLOSEDLOOP_CONTROL

/**********************  support xls file definitions end *********************/
    
    /* Velocity Control Loop Coefficients 
     * has to tuned according to the load characteristics*/    
    #define SPEEDCNTR_PTERM         Q15(0.4)
    #define	SPEEDCNTR_PTERM_SCALE   3
    #define SPEEDCNTR_ITERM         3//Q15(0.00005)
    #define SPEEDCNTR_ITERM_SCALE   0
    #define SPEEDCNTR_OUTMAX        NORM_VALUE(NOMINAL_TORQUE_COMPONENT_CURRENT_PEAK,MC1_PEAK_CURRENT)
  
    /* MRAS estimator PI parameters 
     * has to tuned according to the load characteristics and speed PI*/
    #define MRAS_PTERM       15500 /* Proportinonal Constant */
    #define MRAS_PTERM_SCALE 3    /* Proportinonal Constant scale */
    #define MRAS_ITERM       10   /* Integral Constant */
    
    /* Coefficients for calculating Motor parameter variations in Field weakening */
    #define PARAMETER_UPDATE_COEF1        0/*Default value = 0:- Parameters are varied 
                                             with changes in flux reference(Idref)*/
    #define PARAMETER_UPDATE_COEF1QVALUE 15
    #define PARAMETER_UPDATE_COEF2    32766  

#endif                  
    
    
/**************  support xls file definitions end **************/
    
/* End speed rpm for open loop to closed loop transition */
#define     END_SPEED_RPM       500
    
/* DC bus compensation factor */    
#define DC_BUS_COMP     (int16_t)((float)MC1_BASE_VOLTAGE*32767/MC1_PEAK_VOLTAGE)

/* Maximum voltage square */
#define MAX_VOLTAGE_SQUARE   (int16_t)( (float)VMAX_CLOSEDLOOP_CONTROL*VMAX_CLOSEDLOOP_CONTROL/32767 )

/* Rated current: peak current drawn at rated load */
#define RATED_CURRENT           (float)(NOMINAL_CURRENT_PHASE_RMS*1.414)
#define PEAK_FAULT_CURRENT     NORM_VALUE((RATED_CURRENT*1.3),MC1_PEAK_CURRENT)  
/* Motor Rated Magnetization Current Peak in Amps */
#define NOMINAL_MAGNITIZING_CURRENT_PEAK (float) (NOMINAL_MAGNITIZING_CURRENT_RMS * 1.414) 
  
    
/** Field Weakening Control Parameters */  
#define MIN_MAGNITIZING_CURRENT_PEAK  ((float)NOMINAL_MAGNITIZING_CURRENT_PEAK*0.5)

/*Parameters for voltage feedback based Field weakening control*/   
#define FD_WEAK_VOLTAGE_REF  (int16_t)((float)VMAX_CLOSEDLOOP_CONTROL*0.86) 
#define FD_WEAK_PI_KP               305
#define FD_WEAK_PI_KPSCALE          1
#define FD_WEAK_PI_KI               2
#define FD_WEAK_IDREF_FILT_CONST    350

/*Parameters for inverse speed Field weakening control*/  
#define FD_WEAK_START_SPEED   NORM_VALUE(((float)NOMINAL_SPEED_RPM*0.7), MC1_PEAK_SPEED_RPM)  
    
    
#define     SPEED_RAMP_RATE_COUNT      1 /* Speed change rate in counts */
#define     RAMP_UP_TIME_MULTIPLIER    45 /* Sample time multiplier for up count */
#define     RAMP_DN_TIME_MULTIPLIER    30 /* Sample time multiplier for down count */
/* Speed rampe rate(rpm/sec) = 
 * (SPEED_RAMP_RATE_COUNT/(LOOPTIME_SEC*TIME_MULTIPLIER)) *(MC1_PEAK_SPEED_RPM/32767) */

    
/*Default Parameters used in MRAS estimator */
/* OmegaRotor Low Pass Filter coefficients */
#define KFILTER_OMEGAR   210
/* OmegaSync bilinear Low Pass Filter coefficients */
#define LPF_MRAS_OMEGASYN_XCOEF   998 /* Cutoff frequency = 200Hz */
#define LPF_MRAS_OMEGASYN_YCOEF   30772 
#define DELTA_OMEGA_MAX     3000 /* Maximum delta omega(pu) limit for MRAS PI controller */
    
// </editor-fold>

#ifdef __cplusplus
}
#endif

#endif	/* end of __MC1_USER_PARAMS_H */
