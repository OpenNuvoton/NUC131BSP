/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 7 $
 * $Date: 15/01/15 1:35p $
 * @brief    Capture the PWM1 Channel 0 waveform by PWM0 Channel 0.
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
    if(PWM_GetCaptureIntFlag(PWM0, 0) > 1)
    {
        PWM_ClearCaptureIntFlag(PWM0, 0, PWM_CAPTURE_INT_FALLING_LATCH);
    }
}

/*--------------------------------------------------------------------------------------*/
/* Capture function to calculate the input waveform information                         */
/* u32Count[4] : Keep the internal counter value when input signal rising / falling     */
/*               happens                                                                */
/*                                                                                      */
/* time    A    B     C     D                                                           */
/*           ___   ___   ___   ___   ___   ___   ___   ___                              */
/*      ____|   |_|   |_|   |_|   |_|   |_|   |_|   |_|   |_____                        */
/* index              0 1   2 3                                                         */
/*                                                                                      */
/* The capture internal counter down count from 0x10000, and reload to 0x10000 after    */
/* input signal falling happens (Time B/C/D)                                            */
/*--------------------------------------------------------------------------------------*/
int32_t CalPeriodTime(PWM_T *PWM, uint32_t u32Ch)
{
    uint16_t u32Count[4];
    uint32_t u32i;
    uint16_t u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod;
    uint32_t u32TimeOutCnt;

    /* Clear Capture Falling Indicator (Time A) */
    PWM_ClearCaptureIntFlag(PWM, u32Ch, PWM_CAPTURE_INT_FALLING_LATCH | PWM_CAPTURE_INT_RISING_LATCH);

    /* Wait for Capture Falling Indicator */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(PWM_GetCaptureIntFlag(PWM, u32Ch) < 2)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for PWM Capture Falling Indicator time-out!\n");
            return -1;
        }
    }

    /* Clear Capture Falling Indicator (Time B) */
    PWM_ClearCaptureIntFlag(PWM, u32Ch, PWM_CAPTURE_INT_FALLING_LATCH);

    u32i = 0;

    while(u32i < 4)
    {
        /* Wait for Capture Falling Indicator */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while(PWM_GetCaptureIntFlag(PWM, u32Ch) < 2)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for PWM Capture Falling Indicator time-out!\n");
                return -1;
            }
        }

        /* Clear Capture Falling and Rising Indicator */
        PWM_ClearCaptureIntFlag(PWM, u32Ch, PWM_CAPTURE_INT_FALLING_LATCH | PWM_CAPTURE_INT_RISING_LATCH);

        /* Get Capture Falling Latch Counter Data */
        u32Count[u32i++] = PWM_GET_CAPTURE_FALLING_DATA(PWM, u32Ch);

        /* Wait for Capture Rising Indicator */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while(PWM_GetCaptureIntFlag(PWM, u32Ch) < 1)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for PWM Capture Rising Indicator time-out!\n");
                return -1;
            }
        }

        /* Clear Capture Rising Indicator */
        PWM_ClearCaptureIntFlag(PWM, u32Ch, PWM_CAPTURE_INT_RISING_LATCH);

        /* Get Capture Rising Latch Counter Data */
        u32Count[u32i++] = PWM_GET_CAPTURE_RISING_DATA(PWM, u32Ch);
    }

    u16RisingTime = u32Count[1];

    u16FallingTime = u32Count[0];

    u16HighPeriod = u32Count[1] - u32Count[2];

    u16LowPeriod = 0x10000 - u32Count[1];

    u16TotalPeriod = 0x10000 - u32Count[2];

    printf("\nPWM generate: \nHigh Period=7199 ~ 7201, Low Period=16799 ~ 16801, Total Period=23999 ~ 24001\n");
    printf("\nCapture Result: Rising Time = %d, Falling Time = %d \nHigh Period = %d, Low Period = %d, Total Period = %d.\n\n",
           u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod);
    if((u16HighPeriod < 7199) || (u16HighPeriod > 7201) || (u16LowPeriod < 16799) || (u16LowPeriod > 16801) || (u16TotalPeriod < 23999) || (u16TotalPeriod > 24001))
    {
        printf("Capture Test Fail!!\n");
        return -1;
    }
    else
    {
        printf("Capture Test Pass!!\n");
        return 0;
    }
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
    //CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_PLL, CLK_CLKDIV_HCLK(2));
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HXT, CLK_CLKDIV_HCLK(1));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable PWM module clock */
    CLK_EnableModuleClock(PWM0_MODULE);
    CLK_EnableModuleClock(PWM1_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));

    /* Select PWM module clock source */
    CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL3_PWM0_S_PCLK, 0);
    CLK_SetModuleClock(PWM1_MODULE, CLK_CLKSEL3_PWM1_S_PCLK, 0);

    /* Reset PWM0 and PWM1 */
    SYS_ResetModule(PWM0_RST);
    SYS_ResetModule(PWM1_RST);

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

    /* Set GPA multi-function pins for PWM0 Channel 0 and PWM1 channel 0 */
    SYS->GPA_MFP &= ~(SYS_GPA_MFP_PA12_Msk);
    SYS->GPA_MFP |= SYS_GPA_MFP_PA12_PWM0_CH0;

    SYS->GPA_MFP &= ~(SYS_GPA_MFP_PA2_Msk);
    SYS->GPA_MFP |= SYS_GPA_MFP_PA2_PWM1_CH0;
    SYS->ALT_MFP3 &= ~(SYS_ALT_MFP3_PA2_Msk);
    SYS->ALT_MFP3 |= SYS_ALT_MFP3_PA2_PWM1_CH0;
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
    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("+------------------------------------------------------------------------+\n");
    printf("|                          PWM Driver Sample Code                        |\n");
    printf("|                                                                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will use PWM0 channel 0 to capture\n  the signal from PWM1 channel 0.\n");
    printf("  I/O configuration:\n");
    printf("    PWM0_CH0(PA.12 PWM0 channel 0) <--> PWM1_CH0(PA.2 PWM1 channel 0)\n\n");
    printf("Use PWM0 Channel 0(PA.12) to capture the PWM1 Channel 0(PA.2) Waveform\n");

    while(1)
    {
        printf("Press any key to start PWM Capture Test\n");
        getchar();

        /*--------------------------------------------------------------------------------------*/
        /* Set the PWM1 Channel 0 as PWM output function.                                       */
        /*--------------------------------------------------------------------------------------*/

        /* Assume PWM output frequency is 250Hz and duty ratio is 30%, user can calculate PWM settings by follows.
           duty ratio = (CMR+1)/(CNR+1)
           cycle time = CNR+1
           High level = CMR+1
           PWM clock source frequency = __HXT = 12000000
           (CNR+1) = PWM clock source frequency/prescaler/clock source divider/PWM output frequency
                   = 12000000/2/1/250 = 24000
           (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
           CNR = 23999
           duty ratio = 30% ==> (CMR+1)/(CNR+1) = 30%
           CMR = 7199
           Prescale value is 1 : prescaler= 2
           Clock divider is PWM_CSR_DIV1 : clock divider =1
        */

        /* set PWM1 channel 0 output configuration */
        PWM_ConfigOutputChannel(PWM1, 0, 250, 30);

        /* Enable PWM Output path for PWM1 channel 0 */
        PWM_EnableOutput(PWM1, PWM_CH_0_MASK);

        /* Enable Timer for PWM1 channel 0 */
        PWM_Start(PWM1, PWM_CH_0_MASK);

        /*--------------------------------------------------------------------------------------*/
        /* Set the PWM0 channel 0 for capture function                                          */
        /*--------------------------------------------------------------------------------------*/

        /* If input minimum frequency is 250Hz, user can calculate capture settings by follows.
           Capture clock source frequency = __HXT = 12000000 in the sample code.
           (CNR+1) = Capture clock source frequency/prescaler/clock source divider/minimum input frequency
                   = 12000000/2/1/250 = 24000
           (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
           CNR = 0xFFFF
           (Note: In capture mode, user should set CNR to 0xFFFF to increase capture frequency range.)
        */

        /* set PWM0 channel 0 capture configuration */
        PWM_ConfigCaptureChannel(PWM0, 0, 166, 0);

        /* Enable capture falling edge interrupt for PWM0 channel 0 */
        //PWM_EnableCaptureInt(PWM0, 0, PWM_CAPTURE_INT_FALLING_LATCH);

        /* Enable PWM0 NVIC interrupt */
        //NVIC_EnableIRQ(PWM0_IRQn);

        /* Enable Timer for PWM0 channel 0 */
        PWM_Start(PWM0, PWM_CH_0_MASK);

        /* Enable Capture Function for PWM0 channel 0 */
        PWM_EnableCapture(PWM0, PWM_CH_0_MASK);

        /* Enable falling capture reload */
        PWM0->CAPCTL |= PWM_CAPCTL_FCRLDEN0_Msk;

        /* Wait until PWM0 channel 0 Timer start to count */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while((PWM0->CNT[0]) == 0)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for PWM0 channel 0 Timer start to count time-out!\n");
                goto lexit;
            }
        }

        /* Capture the Input Waveform Data */
        if( CalPeriodTime(PWM0, 0) < 0 ) goto lexit;
        /*---------------------------------------------------------------------------------------------------------*/
        /* Stop PWM1 channel 0 (Recommended procedure method 1)                                                    */
        /* Set PWM Timer loaded value(Period) as 0. When PWM internal counter(CNT) reaches to 0, disable PWM Timer */
        /*---------------------------------------------------------------------------------------------------------*/

        /* Set PWM1 channel 0 loaded value as 0 */
        PWM_Stop(PWM1, PWM_CH_0_MASK);

        /* Wait until PWM1 channel 0 Timer Stop */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while((PWM1->CNT[0] & PWM_CNT_CNT_Msk) != 0)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for PWM1 channel 0 Timer Stop time-out!\n");
                goto lexit;
            }
        }

        /* Disable Timer for PWM1 channel 0 */
        PWM_ForceStop(PWM1, PWM_CH_0_MASK);

        /* Disable PWM Output path for PWM1 channel 0 */
        PWM_DisableOutput(PWM1, PWM_CH_0_MASK);

        /*---------------------------------------------------------------------------------------------------------*/
        /* Stop PWM0 channel 0 (Recommended procedure method 1)                                                    */
        /* Set PWM Timer loaded value(Period) as 0. When PWM internal counter(CNT) reaches to 0, disable PWM Timer */
        /*---------------------------------------------------------------------------------------------------------*/

        /* Disable PWM0 NVIC */
        //NVIC_DisableIRQ(PWM0_IRQn);

        /* Set loaded value as 0 for PWM0 channel 0 */
        PWM_Stop(PWM0, PWM_CH_0_MASK);

        /* Wait until PWM0 channel 0 current counter reach to 0 */
        u32TimeOutCnt = SystemCoreClock;  /* 1 second time-out */
        while((PWM0->CNT[0] & PWM_CNT_CNT_Msk) != 0)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for PWM0 channel 0 current counter reach to 0 time-out!\n");
                goto lexit;
            }
        }

        /* Disable Timer for PWM0 channel 0 */
        PWM_ForceStop(PWM0, PWM_CH_0_MASK);

        /* Disable Capture Function and Capture Input path for PWM0 channel 0 */
        PWM_DisableCapture(PWM0, PWM_CH_0_MASK);

        /* Clear Capture Interrupt flag for PWM0 channel 0 */
        PWM_ClearCaptureIntFlag(PWM0, 0, PWM_CAPTURE_INT_FALLING_LATCH);
    }

lexit:

    while(1);
}




