/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 7 $
 * $Date: 15/01/15 1:29p $
 * @brief    Change duty cycle and period of output waveform by BPWM Double Buffer function.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC131.h"


#define PLLCON_SETTING  CLK_PLLCON_50MHz_HXT
#define PLL_CLOCK       50000000

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
    BPWM0->INTSTS = BPWM_INTSTS_PIF0_Msk;
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK->PWRCON |= CLK_PWRCON_OSC22M_EN_Msk;

    /* Waiting for Internal RC clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_OSC22M_STB_Msk));

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLK_S_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_HIRC;
    CLK->CLKDIV &= ~CLK_CLKDIV_HCLK_N_Msk;
    CLK->CLKDIV |= CLK_CLKDIV_HCLK(1);

    /* Enable external XTAL 12MHz clock */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk;

    /* Waiting for external XTAL clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_XTL12M_STB_Msk));

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCON = PLLCON_SETTING;
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk));
    CLK->CLKSEL0 &= (~CLK_CLKSEL0_HCLK_S_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_PLL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CyclesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For CLK_SysTickDelay()

    /* Enable IP clock */
    CLK->APBCLK = CLK_APBCLK_UART0_EN_Msk;
    CLK->APBCLK1 |= CLK_APBCLK1_BPWM0_EN_Msk;

    /* Select UART module clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART_S_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UART_S_HXT;

    /* select BPWM clock source */
    CLK->CLKSEL3 = CLK_CLKSEL3_BPWM0_S_Msk;

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
    /* Reset UART IP */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_UART0_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART0_RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
    UART0->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
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

    // set BPWM to down count type(edge aligned)
    BPWM0->CTL1 &= ~BPWM_CTL1_CNTTYPE0_Msk;
    BPWM0->CTL1 |= 0x1;

    /*Set BPWM Timer clock prescaler*/
    BPWM_SET_PRESCALER(BPWM0, 0, 0); // Divided by 1

    /*Set BPWM Timer duty*/
    BPWM_SET_CMR(BPWM0, 0, 199);

    /*Set BPWM Timer period*/
    BPWM_SET_CNR(BPWM0, 0, 399);

    /* Set waveform generation */
    BPWM0->WGCTL0 = 0x10000;
    BPWM0->WGCTL1 = 0x20000;

    // Enable output of BPWM0 channel 0
    BPWM0->POEN |= BPWM_POEN_POEN0_Msk;

    // Enable BPWM0 channel 0 period interrupt, use channel 0 to measure time.
    BPWM0->INTEN = (BPWM0->INTEN & ~BPWM_INTEN_PIEN0_Msk) | BPWM_INTEN_PIEN0_Msk;
    NVIC_EnableIRQ(BPWM0_IRQn);

    // Start
    BPWM0->CNTEN |= BPWM_CNTEN_CNTEN0_Msk;

    while(1);
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
