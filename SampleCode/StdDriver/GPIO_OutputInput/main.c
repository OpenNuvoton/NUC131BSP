/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 6 $
 * $Date: 15/01/16 11:44a $
 * @brief    Show how to set GPIO pin mode and use pin data input/output control.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC131.h"

#define PLL_CLOCK   50000000


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD(PB.0) and TXD(PB.1) */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= (SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD);

}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    int32_t i32Err;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-------------------------------------------------+\n");
    printf("|    PB.2(Output) and PC.1(Input) Sample Code     |\n");
    printf("+-------------------------------------------------+\n\n");

    /* Configure PB.2 as Output mode and PC.1 as Input mode */
    GPIO_SetMode(PB, BIT2, GPIO_PMD_OUTPUT);
    GPIO_SetMode(PC, BIT1, GPIO_PMD_INPUT);

    i32Err = 0;
    printf("GPIO PB.2(output mode) connect to PC.1(input mode) ......");

    /* Use Pin Data Input/Output Control to pull specified I/O or get I/O pin status */
    /* Pull PB.2 to Low and check PC.1 status */
    PB2 = 0;
    if(PC1 != 0)
    {
        i32Err = 1;
    }

    /* Pull PB.2 to High and check PC.1 status */
    PB2 = 1;
    if(PC1 != 1)
    {
        i32Err = 1;
    }

    if(i32Err)
    {
        printf("  [FAIL].\n");
    }
    else
    {
        printf("  [OK].\n");
    }

    /* Configure PB.2 and PC.1 to default Quasi-bidirectional mode */
    GPIO_SetMode(PB, BIT2, GPIO_PMD_QUASI);
    GPIO_SetMode(PC, BIT1, GPIO_PMD_QUASI);

    while(1);
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
