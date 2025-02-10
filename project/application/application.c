// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file application.c
 *
 * @brief This module implements application state machine.
 *
 * Component: APPLICATION LOAD
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

#include "application.h"

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">

void MCAPP_ApplicationStateMachine (MCAPP_APPLICATION_T *);
void MCAPP_ApplicationInit (MCAPP_APPLICATION_T *);

// </editor-fold>
/**
* <B> Function: void MCAPP_ApplicationInit (MCAPP_APPLICATION_T *)  </B>
*
* @brief Application load initialization.
*
* @param Pointer to the data structure containing application parameters.
* @return none.
* @example
* <CODE> MCAPP_ApplicationInit(&compr); </CODE>
*
*/
void MCAPP_ApplicationInit (MCAPP_APPLICATION_T *pComp)
{
    pComp->state = APPLICATION_WAIT;
    /*
     
     */
}

/**
* <B> Function: void MCAPP_ApplicationStateMachine (MCAPP_APPLICATION_T *)  </B>
*
* @brief Application state machine.
*
* @param Pointer to the data structure containing application parameters.
* @return none.
* @example
* <CODE> MCAPP_ApplicationStateMachine(&compr); </CODE>
*
*/
void MCAPP_ApplicationStateMachine (MCAPP_APPLICATION_T *pComp)
{
    switch(pComp->state)
    {
        case APPLICATION_WAIT:
            break;
            
        case APPLICATION_RUN:
            /*   */
            break;
            
        case APPLICATION_STOP:
            break;
            
        case APPLICATION_FAULT:            
            break;
            
        default:
            pComp->state = APPLICATION_FAULT;
        break;
    }
}

/**
* <B> Function: void MCAPP_IsApplicationReadyToStart (MCAPP_APPLICATION_T *) </B>
*
* @brief Application Ready to Start Interface Function.
*
* @param Pointer to the data structure containing application parameters.
* @return Ready to start Flag.
* @example
* <CODE> MCAPP_IsApplicationReadyToStart(&compr); </CODE>
*
*/
int16_t MCAPP_IsApplicationReadyToStart (MCAPP_APPLICATION_T *pComp)
{
    return 1;
}

/**
* <B> Function: void MCAPP_IsLoadReadyToStop (MCAPP_APPLICATION_T *)  </B>
*
* @brief Application Ready to Start Interface Function.
*
* @param Pointer to the data structure containing application parameters.
* @return Ready to stop Flag.
* @example
* <CODE> MCAPP_IsLoadReadyToStop(&compr); </CODE>
*
*/
int16_t MCAPP_IsApplicationReadyToStop (MCAPP_APPLICATION_T *pComp)
{
    return 1;
}