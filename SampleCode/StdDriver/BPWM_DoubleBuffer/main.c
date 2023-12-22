/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 15/01/15 1:34p $
 * @brief    Change duty cycle and period of output waveform by BPWM Double Buffer function.
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
 * @brief       BPWM0 IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle BPWM0 interrupt event
 */
void BPWM0_IRQHandler(void)
{
    static int toggle = 0;

    // Update BPWM0 channel 0 period and duty
    if(toggle == 0)
    {
        BPWM_SET_CNR(BPWM0, 0, 99);
        BPWM_SET_CMR(BPWM0, 0, 39);
    }
    else
    {
        BPWM_SET_CNR(BPWM0, 0, 399);
        BPWM_SET_CMR(BPWM0, 0, 199);
    }
    toggle ^= 1;
    // Clear channel 0 period interrupt flag
    BPWM_ClearPeriodIntFlag(BPWM0, 0);
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

    /* Switch HCLK clock source to PLL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_PLL, CLK_CLKDIV_HCLK(2));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));

    /* Enable BPWM0 clock source */
    CLK_EnableModuleClock(BPWM0_MODULE);

    /* Select BPWM module clock source */
    //CLK_SetModuleClock(BPWM0_MODULE, CLK_CLKSEL3_BPWM0_S_HClK, 0);
    CLK_SetModuleClock(BPWM0_MODULE, CLK_CLKSEL3_BPWM0_S_PLL, 0);

    /* Reset BPWM0 */
    SYS_ResetModule(BPWM0_RST);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
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
    /* Set GPC multi-function pins for BPWM0 Channel 0 */
    SYS->GPC_MFP &= ~(SYS_GPC_MFP_PC0_Msk);
    SYS->GPC_MFP |= SYS_GPC_MFP_PC0_BPWM0_CH0;
    SYS->ALT_MFP3 &= ~(SYS_ALT_MFP3_PC0_Msk);
    SYS->ALT_MFP3 |= SYS_ALT_MFP3_PC0_BPWM0_CH0;
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
    printf("|                          BPWM Driver Sample Code                       |\n");
    printf("|                                                                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will use BPWM0 channel 0 to output waveform\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: BPWM0 channel 0(PC.0)\n");
    printf("\nUse double buffer feature.\n");

    /*
        BPWM0 channel 0 waveform of this sample shown below:

        |<-        CNR + 1  clk     ->|  CNR + 1 = 399 + 1 CLKs
                       |<-CMR+1 clk ->|  CMR + 1 = 199 + 1 CLKs
                                      |<-   CNR + 1  ->|  CNR + 1 = 99 + 1 CLKs
                                               |<CMR+1>|  CMR + 1 = 39 + 1 CLKs

      __                ______________          _______
        |______200_____|     200      |____60__|   40  |_____BPWM waveform

    */


    /*
      Configure BPWM0 channel 0 init period and duty.
      Period is __HXT / (prescaler * clock divider * (CNR + 1))
      Duty ratio = (CMR + 1) / (CNR + 1)
      Period = 12 MHz / (2 * 1 * (199 + 1)) =  30000 Hz
      Duty ratio = (99 + 1) / (199 + 1) = 50%
    */
    // BPWM0 channel 0 frequency is 100Hz, duty 30%,
    BPWM_ConfigOutputChannel(BPWM0, 0, 30000, 30);

    // Enable output of BPWM0 channel 0
    BPWM_EnableOutput(BPWM0, BPWM_CH_0_MASK);

    // Enable BPWM0 channel 0 period interrupt, use channel 0 to measure time.
    BPWM_EnablePeriodInt(BPWM0, 0, 0);
    NVIC_EnableIRQ(BPWM0_IRQn);

    // Start
    BPWM_Start(BPWM0, BPWM_CH_0_MASK);

    while(1);

}




