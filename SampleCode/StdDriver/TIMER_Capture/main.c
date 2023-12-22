/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 5 $
 * $Date: 15/01/16 11:44a $
 * @brief    Show how to use the timer0 capture function to capture timer0 counter value.
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


/**
 * @brief       Timer0 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Timer0 default IRQ, declared in startup_NUC131.s.
 */
void TMR0_IRQHandler(void)
{
    if(TIMER_GetCaptureIntFlag(TIMER0) == 1)
    {
        /* Clear Timer0 capture interrupt flag */
        TIMER_ClearCaptureIntFlag(TIMER0);

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
    CLK_EnableModuleClock(TMR2_MODULE);
    CLK_EnableModuleClock(TMR3_MODULE);

    /* Peripheral clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_PLL, CLK_CLKDIV_UART(1));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0_S_HCLK, 0);
    CLK_SetModuleClock(TMR2_MODULE, CLK_CLKSEL1_TMR2_S_HXT, 0);
    CLK_SetModuleClock(TMR3_MODULE, CLK_CLKSEL1_TMR3_S_HXT, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= (SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD);

    /* Set PB multi-function pins for TM0 on PB.8 and TM0_EXT on PB.15 */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB8_Msk | SYS_GPB_MFP_PB15_Msk);
    SYS->GPB_MFP |= (SYS_GPB_MFP_PB8_TM0 | SYS_GPB_MFP_PB15_TM0_EXT);
    SYS->ALT_MFP &= ~(SYS_ALT_MFP_PB15_Msk);
    SYS->ALT_MFP |= SYS_ALT_MFP_PB15_TM0_EXT;

    /* Set PB multi-function pins for TM2 on PB.2 and TM3 on PB.3 */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB2_Msk | SYS_GPB_MFP_PB3_Msk);
    SYS->GPB_MFP |= (SYS_GPB_MFP_PB2_TM2 | SYS_GPB_MFP_PB3_TM3);
    SYS->ALT_MFP2 &= ~(SYS_ALT_MFP2_PB2_Msk | SYS_ALT_MFP2_PB3_Msk);
    SYS->ALT_MFP2 |= (SYS_ALT_MFP2_PB2_TM2 | SYS_ALT_MFP2_PB3_TM3);
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
    uint32_t au32CAPValue[20] = {0}, u32CAPDiff;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("CPU @ %d Hz\n", SystemCoreClock);
    printf("+------------------------------------------+\n");
    printf("|    Timer0 Capture Counter Sample Code    |\n");
    printf("+------------------------------------------+\n\n");

    printf("# Timer0 Settings:\n");
    printf("    - Clock source is HCLK                  \n");
    printf("    - Continuous counting mode              \n");
    printf("    - Interrupt enable                      \n");
    printf("    - Compared value is 0xFFFFFF            \n");
    printf("    - Event counter mode enable on PB.8     \n");
    printf("    - External capture mode enable on PB.15 \n");
    printf("    - Capture trigger interrupt enable      \n");
    printf("# Timer2 Settings:\n");
    printf("    - Clock source is HXT\n");
    printf("    - Time-out frequency is 1000 Hz    			\n");
    printf("    - Toggle-output mode on PB.2 and frequency is 500 Hz\n");
    printf("# Timer3 Settings:\n");
    printf("    - Clock source is HXT\n");
    printf("    - Time-out frequency is 2 Hz    			\n");
    printf("    - Toggle-output mode on PB.3 and frequency is 1 Hz	\n");
    printf("# Connect TM2(PB.2) toggle-output pin to TM0(PB.8) event counter pin.\n");
    printf("# Connect TM3(PB.3) toggle-output pin to TM0_EXT(PB.15) external capture pin.\n\n");

    printf("# Every 500 event counts will be captured when Time0 capture trigger event occurred.\n");

    /* Enable Timer0 NVIC */
    NVIC_EnableIRQ(TMR0_IRQn);

    /* Initial Timer2 and Timer3 default setting */
    TIMER_Open(TIMER2, TIMER_TOGGLE_MODE, 1000);
    TIMER_Open(TIMER3, TIMER_TOGGLE_MODE, 2);

    /* Initial Timer0 default setting */
    TIMER_Open(TIMER0, TIMER_CONTINUOUS_MODE, 1);

    /* Configure Timer0 setting for external counter input and capture function */
    TIMER_SET_PRESCALE_VALUE(TIMER0, 0);
    TIMER_SET_CMP_VALUE(TIMER0, 0xFFFFFF);
    TIMER_EnableEventCounter(TIMER0, TIMER_COUNTER_FALLING_EDGE);
    TIMER_EnableCapture(TIMER0, TIMER_CAPTURE_FREE_COUNTING_MODE, TIMER_CAPTURE_FALLING_EDGE);
    TIMER_EnableCaptureInt(TIMER0);

    /* Clear Timer0 interrupt counts to 0 */
    u32InitCount = g_au32TMRINTCount[0] = 0;

    /* Start Timer0, Timer2 and Timer3 counting */
    TIMER_Start(TIMER0);
    TIMER_Start(TIMER2);
    TIMER_Start(TIMER3);

    /* Check Timer0 capture trigger interrupt counts */
    while(g_au32TMRINTCount[0] < 11)
    {
        if(g_au32TMRINTCount[0] != u32InitCount)
        {
            au32CAPValue[g_au32TMRINTCount[0]] = TIMER_GetCaptureData(TIMER0);
            u32CAPDiff = au32CAPValue[g_au32TMRINTCount[0]] - au32CAPValue[g_au32TMRINTCount[0] - 1];
            printf("    [%2d]: %4d. Diff: %d.\n", g_au32TMRINTCount[0], au32CAPValue[g_au32TMRINTCount[0]], u32CAPDiff);
            if(u32CAPDiff != 500)
            {
                printf("*** FAIL ***\n");
                goto lexit;
            }
            u32InitCount = g_au32TMRINTCount[0];
        }
    }

    printf("*** PASS ***\n");

lexit:

    /* Stop Timer0, Timer2 and Timer3 counting */
    TIMER_Close(TIMER0);
    TIMER_Close(TIMER2);
    TIMER_Close(TIMER3);

    while(1);
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
