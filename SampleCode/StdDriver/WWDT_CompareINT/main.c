/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 7 $
 * $Date: 15/09/14 4:29p $
 * @brief    Show how to reload the WWDT counter value.
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC131.h"

#define PLL_CLOCK           50000000


/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32WWDTINTCount = 0;


/**
 * @brief       IRQ Handler for WDT and WWDT Interrupt
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The WDT_IRQHandler is default IRQ of WDT and WWDT, declared in startup_NUC131.s.
 */
void WDT_IRQHandler(void)
{
    if(WWDT_GET_INT_FLAG() == 1)
    {
        /* Clear WWDT compare match interrupt flag */
        WWDT_CLEAR_INT_FLAG();

        PA0 ^= 1;

        g_u32WWDTINTCount++;

        if(g_u32WWDTINTCount < 10)
        {
            /* To reload the WWDT counter value to 0x3F */
            WWDT_RELOAD_COUNTER();
        }

        printf("WWDT compare match interrupt occurred. (%d)\n", g_u32WWDTINTCount);
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

    /* Set core clock as PLL_CLOCK from PLL and SysTick source to HCLK/2*/
    CLK_SetCoreClock(PLL_CLOCK);
    CLK_SetSysTickClockSrc(CLK_CLKSEL0_STCLK_S_HCLK_DIV2);

    /* Enable peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(WWDT_MODULE);

    /* Peripheral clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_PLL, CLK_CLKDIV_UART(1));
    CLK_SetModuleClock(WWDT_MODULE, CLK_CLKSEL2_WWDT_S_HCLK_DIV2048, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD, TXD */
    SYS->GPB_MFP = SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;
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
    double dPeriodTime;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("CPU @ %d Hz\n", SystemCoreClock);
    printf("+------------------------------------------------+\n");
    printf("|    WWDT Compare March Interrupt Sample Code    |\n");
    printf("+------------------------------------------------+\n\n");

    /* To check if system has been reset by WWDT time-out reset or not */
    if(WWDT_GET_RESET_FLAG() == 1)
    {
        WWDT_CLEAR_RESET_FLAG();
        printf("*** System has been reset by WWDT time-out reset event. [WWDTCR: 0x%08X] ***\n\n", WWDT->WWDTCR);
        while(1);
    }

    dPeriodTime = (((double)(1000000 * 2048) / (double)SystemCoreClock) * 1024) * 32;

    printf("# WWDT Settings: \n");
    printf("    - Clock source is HCLK/2048 (%d Hz)    \n", SystemCoreClock / 2048);
    printf("    - WWDT counter prescale period is 1024, \n");
    printf("        and max WWDT time-out period is 1024 * (64 * WWDT_CLK)\n");
    printf("    - Interrupt enable                      \n");
    printf("    - Window Compare value is 32            \n");
    printf("# System will generate first WWDT compare match interrupt event after %.2f us.\n", dPeriodTime);
    printf("    1.) use PA.0 high/low period to check WWDT compare match interrupt period time\n");
    printf("    2.) reload WWDT counter value to avoid WWDT time-out reset system occurred\n");
    printf("        when interrupt counts less than 11.\n");
    printf("    3.) do not reload WWDT counter value to generate WWDT time-out reset system event\n");
    printf("        when interrupt counts large than 10.\n\n");

    /* Use PA.0 to check WWDT compare match interrupt period time */
    GPIO_SetMode(PA, BIT0, GPIO_PMD_OUTPUT);
    PA0 = 1;

    /* Enable WDT/WWDT NVIC */
    NVIC_EnableIRQ(WDT_IRQn);

    g_u32WWDTINTCount = 0;

    /*
        Max time-out period is 1024*(64*WWDT_CLK);
        WWDT compare value is 32;
        Enable WWDT compare match interrupt;
    */
    /* Note: WWDTCR register can be written only once after chip is powered on or reset */
    WWDT_Open(WWDT_PRESCALER_1024, 32, TRUE);

    printf("[WWDTCR: 0x%08X]\n\n", WWDT->WWDTCR);

    while(1);
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
