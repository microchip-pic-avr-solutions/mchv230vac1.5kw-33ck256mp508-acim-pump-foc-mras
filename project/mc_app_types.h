// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file mc_app_types.h
 *
 * @brief This module initializes data structure variable type definitions of 
 * data structure
 * 
 * Component: MOTOR CONTROL APPLICATION
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

#ifndef MC_APP_TYPES_H
#define	MC_APP_TYPES_H

#ifdef	__cplusplus
extern "C" {
#endif
    
// <editor-fold defaultstate="expanded" desc="ENUMERATED CONSTANTS ">

typedef enum
{
    MCAPP_INIT = 0,                     /* Initialize Run time parameters */
    MCAPP_CMD_WAIT = 1,                 /* Wait for Run command */
    MCAPP_OFFSET = 2,                   /* Measure current offsets */
    MCAPP_LOAD_START_READY_CHECK = 3,   /* Wait for load to be ready to start */
    MCAPP_RUN = 4,                      /* Run the motor */
    MCAPP_LOAD_STOP_READY_CHECK = 5,    /* Wait for load to be ready to stop */
    MCAPP_STOP = 6,                     /* Stop the motor */
    MCAPP_FAULT = 7,                    /* Motor is in Fault mode */

}MCAPP_STATE_T;


// </editor-fold>

#ifdef	__cplusplus
}
#endif

#endif	/* MC_APP_TYPES_H */

