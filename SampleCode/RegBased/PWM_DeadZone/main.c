/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 5 $
 * $Date: 14/06/30 4:50p $
 * @brief    Demonstrate how to use PWM Dead Zone function.
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
    static uint32_t cnt;
    static uint32_t out;

    // Channel 0 frequency is 100Hz, every 1 second enter this IRQ handler 100 times.
    if(++cnt == 100)
    {
        if(out)
            PWM0->POEN |= PWM_POEN_POENn_Msk;
        else
            PWM0->POEN &= ~PWM_POEN_POENn_Msk;
        out ^= 1;
        cnt = 0;
    }
    // Clear channel 0 period interrupt flag
    PWM0->INTSTS0 = PWM_INTSTS0_PIFn_Msk;
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
    CLK->APBCLK1 = CLK_APBCLK_UART0_EN_Msk | CLK_APBCLK1_PWM0_EN_Msk;

    /* Select UART module clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART_S_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UART_S_HXT;

    /* select PWM clock source */
    CLK->CLKSEL3 = CLK_CLKSEL3_PWM0_S_Msk;

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
    printf("  This sample code will output PWM0 channel 0 ~ channel 3 with different\n");
    printf("  frequency and duty, enable dead zone function of all PWM0 pairs.\n");
    printf("  And also enable/disable PWM output every 1 second.\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: PWM0_CH0(PA.12), PWM0_CH1(PA.13), PWM0_CH2(PA.14), PWM0_CH3(PA.15)\n");

    // PWM0 channel0 frequency is 100Hz, duty 30%,
    /* Assume PWM output frequency is 100Hz and duty ratio is 30%, user can calculate PWM settings by follows.
       duty ratio = (CMR+1)/(CNR+1)
       cycle time = CNR+1
       High level = CMR+1
       PWM clock source frequency = __HXT = 12000000
       (CNR+1) = PWM clock source frequency/prescaler/PWM output frequency
               = 12000000/2/100 = 60000
       (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
       CNR = 59999
       duty ratio = 30% ==> (CMR+1)/(CNR+1) = 30% ==> CMR = (CNR+1)*0.3-1 = 60000*30/100-1
       CMR = 17999
       Prescale value is 1 : prescaler= 2
    */

    /*Set Pwm mode as complementary mode*/
    PWM_ENABLE_COMPLEMENTARY_MODE(PWM0);

    /*Set PWM Timer clock prescaler*/
    PWM_SET_PRESCALER(PWM0, 0, 1); // Divided by 2

    /*Set PWM Timer duty*/
    PWM_SET_CMR(PWM0, 0, 17999);
    PWM_SET_CMR(PWM0, 1, 17999);

    /*Set PWM Timer period*/
    PWM_SET_CNR(PWM0, 0, 59999);

    /* Set waveform generation */
    PWM0->WGCTL0 = 0xAAA;
    PWM0->WGCTL1 = 0x555;

    /* enable and configure dead zone */
    SYS_UnlockReg();
    PWM0->DTCTL0_1 = 400;
    PWM0->DTCTL0_1 |= PWM_DTCTL0_1_DTEN_Msk;
    SYS_LockReg();

    // PWM0 channel2 frequency is 300Hz, duty 50%
    /* Assume PWM output frequency is 300Hz and duty ratio is 50%, user can calculate PWM settings by follows.
       duty ratio = (CMR+1)/(CNR+1)
       cycle time = CNR+1
       High level = CMR+1
       PWM clock source frequency = __HXT = 12000000
       (CNR+1) = PWM clock source frequency/prescaler/PWM output frequency
               = 12000000/2/300 = 20000
       (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
       CNR = 19999
       duty ratio = 50% ==> (CMR+1)/(CNR+1) = 50% ==> CMR = (CNR+1)*0.5-1 = 20000*50/100-1
       CMR = 9999
       Prescale value is 1 : prescaler= 2
    */

    /*Set PWM0 Timer clock prescaler*/
    PWM_SET_PRESCALER(PWM0, 2, 1); // Divided by 2

    /*Set PWM0 Timer duty*/
    PWM_SET_CMR(PWM0, 2, 9999);
    PWM_SET_CMR(PWM0, 3, 9999);

    /*Set PWM0 Timer period*/
    PWM_SET_CNR(PWM0, 2, 19999);

    /* enable and configure dead zone */
    SYS_UnlockReg();
    PWM0->DTCTL2_3 = 200;
    PWM0->DTCTL2_3 |= PWM_DTCTL2_3_DTEN_Msk;
    SYS_LockReg();

    // Enable output of all PWM0 channels
    PWM0->POEN |= PWM_POEN_POENn_Msk;

    // Enable PWM0 channel 0 period interrupt, use channel 0 to measure time.
    PWM0->INTEN0 = (PWM0->INTEN0 & ~PWM_INTEN0_PIENn_Msk) | PWM_INTEN0_PIENn_Msk;
    NVIC_EnableIRQ(PWM0_IRQn);

    // Start
    PWM0->CNTEN = 0xF;

    while(1);
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
