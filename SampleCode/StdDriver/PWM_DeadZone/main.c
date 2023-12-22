/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 4 $
 * $Date: 14/06/30 4:51p $
 * @brief    Demonstrate how to use PWM Dead Zone function.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC131.h"


#define PLL_CLOCK       50000000

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
    static uint32_t cnt;
    static uint32_t out;

    // Channel 0 frequency is 100Hz, every 1 second enter this IRQ handler 100 times.
    if(++cnt == 100)
    {
        if(out)
            PWM_EnableOutput(PWM0, PWM_CH_0_MASK | PWM_CH_1_MASK | PWM_CH_2_MASK | PWM_CH_3_MASK);
        else
            PWM_DisableOutput(PWM0, PWM_CH_0_MASK | PWM_CH_1_MASK | PWM_CH_2_MASK | PWM_CH_3_MASK);
        out ^= 1;
        cnt = 0;
    }
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

    /* Switch HCLK clock source to PLL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_PLL, CLK_CLKDIV_HCLK(2));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable PWM0 clock source */
    CLK_EnableModuleClock(PWM0_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));

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
    /* Set GPA multi-function pins for PWM0 Channel 0 ~ channel 3 */
    SYS->GPA_MFP &= ~(SYS_GPA_MFP_PA12_Msk | SYS_GPA_MFP_PA13_Msk | SYS_GPA_MFP_PA14_Msk | SYS_GPA_MFP_PA15_Msk);
    SYS->GPA_MFP |= (SYS_GPA_MFP_PA12_PWM0_CH0 | SYS_GPA_MFP_PA13_PWM0_CH1 | SYS_GPA_MFP_PA14_PWM0_CH2 | SYS_GPA_MFP_PA15_PWM0_CH3);
    SYS->ALT_MFP4 &= ~(SYS_ALT_MFP4_PA12_Msk);
    SYS->ALT_MFP4 &= ~(SYS_ALT_MFP4_PA13_Msk);
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
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART0_Init();
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          PWM Driver Sample Code                        |\n");
    printf("|                                                                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will output PWM0 channel 0~3 with different\n");
    printf("  frequency and duty, enable dead zone function of all PWM0 pairs.\n");
    printf("  And also enable/disable PWM output every 1 second.\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: PWM0_CH0(PA.12), PWM0_CH1(PA.13), PWM0_CH2(PA.14), PWM0_CH3(PA.15)\n");

    /*Set Pwm mode as complementary mode*/
    PWM_ENABLE_COMPLEMENTARY_MODE(PWM0);

    // PWM0 channel 0 frequency is 100Hz, duty 30%,
    PWM_ConfigOutputChannel(PWM0, 0, 100, 30);
    SYS_UnlockReg();
    PWM_EnableDeadZone(PWM0, 0, 400);
    SYS_LockReg();

    // PWM0 channel 2 frequency is 300Hz, duty 50%
    PWM_ConfigOutputChannel(PWM0, 2, 300, 50);
    SYS_UnlockReg();
    PWM_EnableDeadZone(PWM0, 2, 200);
    SYS_LockReg();

    // Enable output of PWM0 channel 0~3
    PWM_EnableOutput(PWM0, 0xF);

    // Enable PWM0 channel 0 period interrupt, use channel 0 to measure time.
    PWM_EnablePeriodInt(PWM0, 0, 0);
    NVIC_EnableIRQ(PWM0_IRQn);

    // Start
    PWM_Start(PWM0, PWM_CH_0_MASK | PWM_CH_2_MASK);

    while(1);
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
