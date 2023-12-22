/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 8 $
 * $Date: 15/01/15 1:30p $
 * @brief    Capture the PWM1 Channel 0 waveform by PWM0 Channel 0.
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
    uint32_t u32CapIntFlag;

    u32CapIntFlag = PWM0->CAPIF;

    if(u32CapIntFlag & PWM_CAPIF_CFLIF0_Msk)
    {
        PWM0->CAPIF = PWM_CAPIF_CFLIF0_Msk;
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
int32_t CalPeriodTime()
{
    uint16_t u32Count[4];
    uint32_t u32i;
    uint16_t u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod;
    uint32_t u32TimeOutCnt;

    /* Clear Capture Falling Indicator (Time A) */
    PWM0->CAPIF = PWM_CAPIF_CFLIF0_Msk;

    /* Wait for Capture Falling Indicator */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while((PWM0->CAPIF & PWM_CAPIF_CFLIF0_Msk) == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for PWM Capture Falling Indicator time-out!\n");
            return -1;
        }
    }

    /* Clear Capture Falling Indicator (Time B) */
    PWM0->CAPIF = PWM_CAPIF_CFLIF0_Msk;

    u32i = 0;

    while(u32i < 4)
    {
        /* Wait for Capture Falling Indicator */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while((PWM0->CAPIF & PWM_CAPIF_CFLIF0_Msk) == 0)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for PWM Capture Falling Indicator time-out!\n");
                return -1;
            }
        }

        /* Clear Capture Falling and Rising Indicator */
        PWM0->CAPIF = PWM_CAPIF_CFLIF0_Msk | PWM_CAPIF_CRLIF0_Msk;

        /* Get Capture Falling Latch Counter Data */
        u32Count[u32i++] = PWM_GET_CAPTURE_FALLING_DATA(PWM0, 0);

        /* Wait for Capture Rising Indicator */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while((PWM0->CAPIF & PWM_CAPIF_CRLIF0_Msk) == 0)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for PWM Capture Rising Indicator time-out!\n");
                return -1;
            }
        }

        /* Clear Capture Rising Indicator */
        PWM0->CAPIF = PWM_CAPIF_CRLIF0_Msk;

        /* Get Capture Rising Latch Counter Data */
        u32Count[u32i++] = PWM_GET_CAPTURE_RISING_DATA(PWM0, 0);
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
    CLK->APBCLK1 = CLK_APBCLK1_PWM0_EN_Msk | CLK_APBCLK1_PWM1_EN_Msk;

    /* Select UART module clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART_S_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UART_S_HXT;

    /* select PWM clock source */
    CLK->CLKSEL3 = CLK_CLKSEL3_PWM0_S_Msk | CLK_CLKSEL3_PWM1_S_Msk;

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
        printf("\n\nPress any key to start PWM Capture Test\n");
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

        /*Set counter as down count*/
        PWM1->CTL1 = (PWM1->CTL1 & ~PWM_CTL1_CNTTYPE0_Msk) | 0x1;

        /*Set PWM Timer clock prescaler*/
        PWM_SET_PRESCALER(PWM1, 0, 1); // Divided by 2

        /*Set PWM Timer duty*/
        PWM_SET_CMR(PWM1, 0, 7199);

        /*Set PWM Timer period*/
        PWM_SET_CNR(PWM1, 0, 23999);

        /* Set waveform generation */
        PWM1->WGCTL0 = 0x00010000;
        PWM1->WGCTL1 = 0x00020000;

        /* Enable PWM Output path for PWM1 channel 0 */
        PWM1->POEN |= PWM_CH_0_MASK;

        /* Enable Timer for PWM1 channel 0 */
        PWM1->CNTEN |= PWM_CH_0_MASK;

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

        /*Set counter as down count*/
        PWM0->CTL1 = (PWM0->CTL1 & ~PWM_CTL1_CNTTYPE0_Msk) | (0x1 << PWM_CTL1_CNTTYPE0_Pos);

        /*Set PWM0 channel 0 Timer clock prescaler*/
        PWM_SET_PRESCALER(PWM0, 0, 1); // Divided by 2

        /*Set PWM0 channel 0 Timer period*/
        PWM_SET_CNR(PWM0, 0, 0xFFFF);

        /* Enable capture falling edge interrupt for PWM0 channel 0 */
        //PWM0->CAPIEN |= PWM_CAPIEN_CAPFIEN0_Msk;

        /* Enable capture function */
        PWM0->CAPCTL |= PWM_CAPCTL_CAPEN0_Msk;

        /* Enable falling capture reload */
        PWM0->CAPCTL |= PWM_CAPCTL_FCRLDEN0_Msk;

        //NVIC_EnableIRQ(PWM0_IRQn);

        // Start
        PWM0->CNTEN |= PWM_CNTEN_CNTEN0_Msk;

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

        /* Enable capture input path for PWM0 channel 0 */
        PWM0->CAPINEN |= PWM_CAPINEN_CAPINEN0_Msk;

        /* Capture the Input Waveform Data */
        if( CalPeriodTime() < 0 ) goto lexit;
        /*---------------------------------------------------------------------------------------------------------*/
        /* Stop PWM1 channel 0 (Recommended procedure method 1)                                                    */
        /* Set PWM Timer loaded value(Period) as 0. When PWM internal counter(CNT) reaches to 0, disable PWM Timer */
        /*---------------------------------------------------------------------------------------------------------*/

        /* Set PWM1 channel 0 loaded value as 0 */
        PWM1->PERIOD[0] = 0;

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
        PWM1->CNTEN &= ~PWM_CNTEN_CNTEN0_Msk;

        /* Disable PWM Output path for PWM1 channel 0 */
        PWM1->POEN &= ~PWM_CH_0_MASK;

        /*---------------------------------------------------------------------------------------------------------*/
        /* Stop PWM0 channel 0 (Recommended procedure method 1)                                                    */
        /* Set PWM Timer loaded value(Period) as 0. When PWM internal counter(CNT) reaches to 0, disable PWM Timer */
        /*---------------------------------------------------------------------------------------------------------*/

        /* Disable PWM0 NVIC */
        //NVIC_DisableIRQ(PWM0_IRQn);

        /* Set loaded value as 0 for PWM0 channel 0 */
        PWM0->PERIOD[0] = 0;

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
        PWM0->CNTEN &= ~PWM_CNTEN_CNTEN0_Msk;

        /* Disable Capture Function and Capture Input path for PWM0 channel 0 */
        PWM0->CAPCTL &= ~PWM_CAPCTL_CAPEN0_Msk;
        PWM0->CAPINEN &= ~PWM_CAPINEN_CAPINEN0_Msk;

        /* Clear Capture Interrupt flag for PWM0 channel 0 */
        PWM0->CAPIF = PWM_CAPIF_CFLIF0_Msk;
    }

lexit:

    while(1);
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
