// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file general.h
 *
 * @brief This file has general definitions and macros used in the application
 *        firmware.
 *
 * Component: APPLICATION
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
#ifndef __GENERAL_H
#define __GENERAL_H

#ifdef __cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">
#include <stdint.h>

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="DEFINITIONS/MACROS ">
/* PI */
#define PI          3.1415927

/* Normalized value of P1(3.14) by 2,where PI = 32767 */
#define PI_BY_2     16384
    
/* Macro to convert a floating point number to Q15 format fixed point number */
#define Q15(floatValue)	\
            ((floatValue < 0.0) ? (int16_t)(32768 * (floatValue) - 0.5): \
            (int16_t)(32767 * (floatValue) + 0.5))

/* Function to calculate normalized value of a parameter */
#define NORM_VALUE(actual, base)    Q15( ((float)actual) / ((float)base) )

    
/**
 * Computes saturated shift-right with an S16 result.
 * If (x >> q) lies within the range of int16_t values, return it,
 * otherwise return the appropriately saturated result (0x8000 if x is negative,
 * else 0x7fff).
 */
inline static int16_t UTIL_SatShrS16(int32_t x, uint16_t q)
{
    const int32_t y = x >> q;    
    const int16_t ylo = y;
    if (q < 16)       // the only chance of overflow is for shift counts < 16
    {        
        const int16_t yhi = y >> 16;
        /* unused bits that will be thrown away
         * These must match the sign of y: either all zero or all one
         */
        const int16_t sign_ylo = ylo >> 15; // -1 if ylo is negative, otherwise 0
        if (yhi != sign_ylo)
        {
            // Uh oh, we had an overflow and need to saturate!
            const int16_t xhi = x >> 16;
            const int16_t sign_x = xhi >> 15;
            return sign_x ^ 0x7fff;
        }
    }
    return ylo;
}    
    
    
    
    
// </editor-fold>

#ifdef __cplusplus
}
#endif

#endif	/* end of __GENERAL_H */
