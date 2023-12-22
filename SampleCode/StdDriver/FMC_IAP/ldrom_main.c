/******************************************************************************
 * @file     LDROM_main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 14/06/06 10:01a $
 * @brief    FMC LDROM IAP sample program for NUC131 series MCU
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NUC131.h"


#ifdef __GNUC__
#define printf(...)
#endif

#define PLL_CLOCK           50000000

void ProcessHardFault(void){}

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

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;
}

void UART_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    UART_Open(UART0, 115200);
}


/**
 * @brief    Routine to get a char
 * @param    None
 * @returns  Get value from UART debug port or semihost
 * @details  Wait UART debug port or semihost to input a char.
 */
static char GetChar(void)
{
    while(1)
    {
        if ((UART0->FSR & UART_FSR_RX_EMPTY_Msk) == 0)
        {
            return (UART0->DATA);
        }
    }
}

/*
 * @returns     Send value from UART debug port
 * @details     Send a target char to UART debug port .
 */
static void SendChar_ToUART(int ch)
{
    while (UART0->FSR & UART_FSR_TX_FULL_Msk);

    UART0->DATA = ch;
    if(ch == '\n')
    {
        while (UART0->FSR & UART_FSR_TX_FULL_Msk);
        UART0->DATA = '\r';
    }
}

static void PutString(char *str)
{
    while (*str != '\0')
    {
        SendChar_ToUART(*str++);
    }
}


int main()
{
    uint32_t u32TimeOutCnt;

    /* Unlock protected register */
    SYS_UnlockReg();

    SYS_Init();
    UART_Init();

    PutString("\n\n");
    PutString("NUC131 FMC IAP Sample Code [LDROM code]\n");

    /* Enable FMC ISP function */
    FMC_Open();

    PutString("\n\nPress any key to branch to APROM...\n");
    GetChar();

    PutString("\n\nChange VECMAP and branch to APROM...\n");
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    UART_WAIT_TX_EMPTY(UART0)
        if(--u32TimeOutCnt == 0) break;

    /* Mask all interrupt before changing VECMAP to avoid wrong interrupt handler fetched */
    __set_PRIMASK(1);

    /* Change VECMAP for booting to APROM */
    FMC_SetVectorPageAddr(FMC_APROM_BASE);

    /* Lock protected Register */
    SYS_LockReg();

    /* Software reset to boot to APROM */
    NVIC_SystemReset();

    while(1);
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
