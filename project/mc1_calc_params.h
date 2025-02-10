// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file mc1_calc_params.h
 *
 * @brief This file has definitions used in the application to run motor 1,
 *        calculated based on associated user parameter header file
 *        mc1_user_params.h.
 *
 * Component: BOARD
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

#ifndef __MC1_CALC_PARAMS_H
#define __MC1_CALC_PARAMS_H

#ifdef __cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>

#include "board_service.h"
#include "foc/general.h"
#include "mc1_user_params.h"

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="DEFINITIONS/MACROS ">
    
#define MC1_LOOPTIME_TCY        LOOPTIME_TCY 
    
#define MC1_LOOPTIME_SEC        LOOPTIME_SEC
    
    
#define MC1_NORM_DELTAT         NORM_DELTA_T
    
    




// </editor-fold>

#ifdef __cplusplus
}
#endif

#endif	/* end of __MC1_CALC_PARAMS_H */
