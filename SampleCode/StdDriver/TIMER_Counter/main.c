/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 6 $
 * $Date: 15/09/14 4:28p $
 * @brief    Implement timer0 event counter function to count the external input event
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC131.h"

#define PLL_CLOCK           50000000


/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_au32TMRINTCount[4] = {0};


/*---------------------------------------------------------------------------------------------------------*/
/*  Generate Event Counter Source by specify GPIO pin                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void GenerateEventCounterSource(uint32_t u32Port, uint32_t u32Pin, uint32_t u32Counts)
{
    while(u32Counts--)
    {
        GPIO_PIN_DATA(u32Port, u32Pin) = 1;
        GPIO_PIN_DATA(u32Port, u32Pin) = 0;
    }
}

/**
  * @brief      Timer0 IRQ
  *
  * @param      None
  *
  * @return     None
  *
  * @details    The Timer0 default IRQ, declared in startup_NUC131.s.
  */
void TMR0_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER0) == 1)
    {
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER0);

        g_au32TMRINTCount[0]++;
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCON_IRC22M_EN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_IRC22M_STB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

    /* Enable HXT */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Set core clock as PLL_CLOCK from PLL and SysTick source to HCLK/2 */
    CLK_SetCoreClock(PLL_CLOCK);
    CLK_SetSysTickClockSrc(CLK_CLKSEL0_STCLK_S_HCLK_DIV2);

    /* Enable peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Peripheral clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_PLL, CLK_CLKDIV_UART(1));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0_S_HCLK, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD, TXD and TM0 counter input pin */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk | SYS_GPB_MFP_PB8_Msk);
    SYS->GPB_MFP |= SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD | SYS_GPB_MFP_PB8_TM0;
}

void UART0_Init(void)
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
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    volatile uint32_t u32InitCount;
    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("CPU @ %d Hz\n", SystemCoreClock);
    printf("+-------------------------------------------------+\n");
    printf("|    Timer0 External Counter Input Sample Code    |\n");
    printf("+-------------------------------------------------+\n\n");

    printf("# Timer0 Settings:\n");
    printf("    - Clock source is HCLK      \n");
    printf("    - Continuous counting mode  \n");
    printf("    - Interrupt enable          \n");
    printf("    - Event counter mode enable \n");
    printf("    - Compared value is 56789   \n");
    printf("# Connect PA.8 pin to event counter pin TM0(PB.8) and pull PA.8 High/Low to generate TM0 counter input source.\n\n");

    /* Configure PA.8 as GPIO output pin and pull pin status to Low first */
    GPIO_SetMode(PA, BIT8, GPIO_PMD_OUTPUT);
    PA8 = 0;

    /* Enable Timer0 NVIC */
    NVIC_EnableIRQ(TMR0_IRQn);

    /* Clear Timer0 interrupt counts to 0 */
    g_au32TMRINTCount[0] = 0;

    /* Configure Timer0 settings and for event counter application */
    TIMER_Open(TIMER0, TIMER_CONTINUOUS_MODE, 1);
    TIMER_SET_PRESCALE_VALUE(TIMER0, 0);
    TIMER_SET_CMP_VALUE(TIMER0, 56789);
    TIMER_EnableEventCounter(TIMER0, TIMER_COUNTER_FALLING_EDGE);
    TIMER_EnableInt(TIMER0);

    /* Start Timer0 counting */
    TIMER_Start(TIMER0);

    /* To check if counter value of Timer0 should be 0 while event counter mode is enabled */
    if(TIMER_GetCounter(TIMER0) != 0)
    {
        printf("Default counter value is not 0. (%d)\n", TIMER_GetCounter(TIMER0));
        goto lexit;
    }

    printf("Start to check Timer0 counter input value ......\n\n");

    /* To generate one counter event from PA.8 to TM0 pin */
    GenerateEventCounterSource(0, 8, 1);

    /* To check if counter value of Timer0 should be 1 */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(TIMER_GetCounter(TIMER0) == 0)
        if(--u32TimeOutCnt == 0) break;
    if(TIMER_GetCounter(TIMER0) != 1)
    {
        printf("Get unexpected counter value. (%d)\n", TIMER_GetCounter(TIMER0));
        goto lexit;
    }

    /* To generate remains counts to TM0 pin */
    GenerateEventCounterSource(0, 8, (56789 - 1));

    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(1)
    {
        if(g_au32TMRINTCount[0] == 1)
        {
            printf("# Timer0 interrupt event occurred.\n");
            break;
        }

        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for Timer0 interrupt time-out!\n");
            goto lexit;
        }
    }

    printf("# Get Timer0 event counter value is %d .... ", TIMER_GetCounter(TIMER0));
    if(TIMER_GetCounter(TIMER0) == 56789)
    {
        printf("PASS.\n");
    }
    else
    {
        printf("FAIL.\n");
    }

lexit:

    /* Stop Timer0 counting */
    TIMER_Close(TIMER0);

    while(1);
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
