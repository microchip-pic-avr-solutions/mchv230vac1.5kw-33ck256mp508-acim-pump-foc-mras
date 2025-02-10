// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file main.c
 *
 * @brief This is the main entry to the application.
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

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>
#include <stdbool.h>

#include <xc.h>

#include "board_service.h"

#include "diagnostics/diagnostics.h"

#include "mc1_service.h"



// </editor-fold>

// <editor-fold defaultstate="collapsed" desc=" Global Variables ">


int16_t runCmdMC1, qTargetVelocityMC1;
// </editor-fold>

/**
* <B> Function: int main (void)  </B>
*
* @brief main() function,entry point of the application.
*
*/
int main (void)
{
    InitOscillator();
    SetupGPIOPorts();
    
#ifdef ENABLE_DIAGNOSTICS
    DiagnosticsInit();
#endif
    HAL_InitPeripherals();

    MCAPP_MC1ServiceInit();
    LED1 = 1;
    HAL_ResetPeripherals();
    runCmdMC1  = 0;
    while(1)
    {
        
#ifdef ENABLE_DIAGNOSTICS
        DiagnosticsStepMain();
#endif
        BoardService();
            
            if(IsPressed_Button1())
            {
                if(runCmdMC1 == 1)
                {
                    runCmdMC1 = 0;
                }
                else
                {
                    runCmdMC1 = 1;
                }

            }
            LED2 = runCmdMC1;
    }
}



void __attribute__((__interrupt__,__auto_psv__)) _T1Interrupt (void)
{
    qTargetVelocityMC1 = MCAPP_MC1GetTargetVelocity();
    MCAPP_MC1InputBufferSet(runCmdMC1, qTargetVelocityMC1);
    BoardServiceStepIsr();
    TIMER1_InterruptFlagClear();
}
