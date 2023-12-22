/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 7 $
 * $Date: 15/01/15 1:36p $
 * @brief    Change duty cycle and period of output waveform by PWM Double Buffer function.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC131.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/

#define PLL_CLOCK           50000000


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/**
 * @brief       PWM0 IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle PWM0 interrupt event
 */
void PWM0_IRQHandler(void)
{
    static int toggle = 0;

    // Update PWM0 channel 0 period and duty
    if(toggle == 0)
    {
        PWM_SET_CNR(PWM0, 0, 99);
        PWM_SET_CMR(PWM0, 0, 39);
    }
    else
    {
        PWM_SET_CNR(PWM0, 0, 399);
        PWM_SET_CMR(PWM0, 0, 199);
    }
    toggle ^= 1;
    // Clear channel 0 period interrupt flag
    PWM_ClearPeriodIntFlag(PWM0, 0);
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC clock */
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for IRC22M clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

    /* Enable external 12MHz XTAL, internal 22.1184MHz */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk | CLK_PWRCON_OSC22M_EN_Msk);

    /* Enable PLL and Set PLL frequency */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_PLL_STB_Msk | CLK_CLKSTATUS_XTL12M_STB_Msk | CLK_CLKSTATUS_OSC22M_STB_Msk);

    /* Switch HCLK clock source to PLL, STCLK to HCLK/2 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_PLL, CLK_CLKDIV_HCLK(2));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));

    /* Enable PWM0 module clock */
    CLK_EnableModuleClock(PWM0_MODULE);

    /* Select PWM module clock source */
    //CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL3_PWM0_S_HClK, 0);
    CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL3_PWM0_S_PLL, 0);

    /* Reset PWM0 */
    SYS_ResetModule(PWM0_RST);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CyclesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For CLK_SysTickDelay()

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= (SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD);
    /* Set GPA multi-function pins for PWM0 Channel 0 */
    SYS->GPA_MFP &= ~(SYS_GPA_MFP_PA12_Msk);
    SYS->GPA_MFP |= (SYS_GPA_MFP_PA12_PWM0_CH0);
    SYS->ALT_MFP4 &= ~(SYS_ALT_MFP4_PA12_Msk);
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART0_Init();
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          PWM Driver Sample Code                        |\n");
    printf("|                                                                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will use PWM0 channel 0 to output waveform\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: PWM0 channel 0(PA.12)\n");
    printf("\nUse double buffer feature.\n");

    /*
        PWM0 channel 0 waveform of this sample shown below:

        |<-        CNR + 1  clk     ->|  CNR + 1 = 399 + 1 CLKs
                       |<-CMR+1 clk ->|  CMR + 1 = 199 + 1 CLKs
                                      |<-   CNR + 1  ->|  CNR + 1 = 99 + 1 CLKs
                                               |<CMR+1>|  CMR + 1 = 39 + 1 CLKs
      __                ______________          _______
        |______200_____|     200      |____60__|   40  |_____PWM waveform

    */


    /*
      Configure PWM0 channel 0 init period and duty.
      Period is __HXT / (prescaler * clock divider * (CNR + 1))
      Duty ratio = (CMR + 1) / (CNR + 1)
      Period = 12 MHz / (2 * 1 * (199 + 1)) =  30000 Hz
      Duty ratio = (99 + 1) / (199 + 1) = 50%
    */
    // PWM0 channel 0 frequency is 100Hz, duty 30%,
    PWM_ConfigOutputChannel(PWM0, 0, 30000, 30);

    // Enable output of PWM0 channel 0
    PWM_EnableOutput(PWM0, PWM_CH_0_MASK);

    // Enable PWM0 channel 0 period interrupt, use channel 0 to measure time.
    PWM_EnablePeriodInt(PWM0, 0, 0);
    NVIC_EnableIRQ(PWM0_IRQn);

    // Start
    PWM_Start(PWM0, PWM_CH_0_MASK);

    while(1);

}




