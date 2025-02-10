// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file sat_PI.h
 *
 * @brief This module implements Proportional Integral(PI) Control with 
 * antiwindup saturation.
 *
 * Component: PI
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


#ifndef __MCAPP_SAT_PI_H
#define __MCAPP_SAT_PI_H

#ifdef __cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include <stdbool.h>

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="VARIABLE TYPE DEFINITIONS ">


/**
 * Status of saturation
 */
typedef enum tagSatState
{
    MCAPP_SAT_NONE = 0,             /** No saturation indicator */
    MCAPP_SAT_VOLT = 1,             /** Voltage saturation state indicator */
    MCAPP_SAT_CURRENT = 2,          /** Current saturation state indicator */
    MCAPP_SAT_VOLT_AND_CURRENT = 3  /** Voltage and Current saturation state indicator */
} MCAPP_SAT_STATE_T;



/**
 * State variables related to PI controller 
 */
typedef struct
{
    int32_t integrator; /** Integrator sum */
    int16_t kp;         /** Proportional gain coefficient term */
    int16_t ki;         /** Integral gain coefficient term */
    int16_t kc;         /** Antiwindup gain coefficient term */
    int16_t nkp;        /** Normalizing term for proportional coefficient */
    int16_t nki;        /** Normalizing term for integral coefficient */
    int16_t outMax;     /** Maximum output limit */
    int16_t outMin;     /** Minimum output limit */
} MCAPP_PISTATE_T;


// </editor-fold>


/**
 * Computes PI correction. 
 * This function computes the output of a modified PI controller. The
 * proportional term is calculated and saturated, and the integrator term
 * with anti-windup is conditionally updated under conditions determined
 * by the sat_State, inReference, inMeasure and direction inputs.
 * 
 * The integrator term is conditionally updated when either of two 
 * conditions are true:
 * - sat_State is MCAPP_SAT_NONE
 * - the error term and the direction are opposite polarities, that is:
 *      inReference <= inMeasure when direction >= 0
 *   or inReference >= inMeasure when direction < 0.
 * 
 * Summary : Modified PI controller
 * 
 * @param inReference Reference Input
 * @param inMeasure Measured Input
 * @param state PI controller state variables
 * @param sat_State status of saturation state to support anti-windup 
 * @param out output of PI controller where result is stored
 * @param direction sign of command, >=0 for positive, <0 for negative
 */
void MCAPP_ControllerPIUpdate(int16_t inReference, int16_t inMeasure, 
        MCAPP_PISTATE_T *state, MCAPP_SAT_STATE_T sat_State, int16_t *out,
        int16_t direction);

/**
 *  Initialize PI controller. 

 * Summary : Initialize PI controller integrator
 * 
 * @param state PI controller state variables 
 */
void MCAPP_ControllerPIInit(MCAPP_PISTATE_T *state);

/**
 *  Reset PI controller. 

 * Summary : Reset PI controller integrator
 * 
 * @param state PI controller state variables
 * @param value 
 */
void MCAPP_ControllerPIReset(MCAPP_PISTATE_T *state, int16_t value);

/**
 * Returns a directed version of (a <= b). If dir is negative, we reverse the
 * sign and compute a >= b instead.
 * 
 * @param a first operand
 * @param b second operand
 * @param dir direction
 * @return a <= b if dir is nonnegative else a >= b
 */
inline static bool UTIL_DirectedLessThanEqual(int16_t a, int16_t b, int16_t dir)
{
    return (dir >= 0)
         ? (a <= b)
         : (a >= b);
}


/** * Limits the input between a minimum and a maximum */
inline static int16_t UTIL_LimitS16(int16_t x, int16_t min, int16_t max)
{
    return (x > max ) ? max : ((x < min) ? min : x);
}



#ifdef __cplusplus
}
#endif

#endif /* __MCAPP_SAT_PI_H */