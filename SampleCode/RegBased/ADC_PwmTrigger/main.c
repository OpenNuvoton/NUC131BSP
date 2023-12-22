/****************************************************************************
 * @file     main.c
 * @version  V2.0
 * $Revision: 5 $
 * $Date: 14/06/30 4:50p $
 * @brief    Demonstrate how to trigger ADC by PWM.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC131.h"

#define PLLCON_SETTING  CLK_PLLCON_50MHz_HXT
#define PLL_CLOCK       50000000


/*---------------------------------------------------------------------------------------------------------*/
/* Define Function Prototypes                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
void UART0_Init(void);
void ADC_PWMTrigTest_SingleOpMode(void);


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK->PWRCON |= CLK_PWRCON_OSC22M_EN_Msk;

    /* Waiting for Internal RC clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_OSC22M_STB_Msk));

    /* Switch HCLK clock source to Internal RC */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLK_S_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_HIRC;

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

    /* Enable UART module clock */
    CLK->APBCLK |= CLK_APBCLK_UART0_EN_Msk;

    /* Enable ADC module clock */
    CLK->APBCLK |= CLK_APBCLK_ADC_EN_Msk ;

    /* Enable PWM01 module clock */
    CLK->APBCLK1 |= CLK_APBCLK1_PWM0_EN_Msk | CLK_APBCLK1_PWM1_EN_Msk;

    /* Select UART module clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART_S_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UART_S_HXT;

    /* Select ADC module clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_ADC_S_Msk ;
    CLK->CLKSEL1 |= CLK_CLKSEL1_ADC_S_HIRC ;

    /* select PWM clock source */
    CLK->CLKSEL3 = CLK_CLKSEL3_PWM0_S_Msk | CLK_CLKSEL3_PWM1_S_Msk;

    /* ADC clock source is 22.1184MHz, set divider to 7, ADC clock is 22.1184/7 MHz */
    CLK->CLKDIV  = (CLK->CLKDIV & ~CLK_CLKDIV_ADC_N_Msk) | (((7) - 1) << CLK_CLKDIV_ADC_N_Pos);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;

    /* Disable the GPA0 - GPA3 digital input path to avoid the leakage current. */
    PA->OFFD = 0xF << GPIO_OFFD_OFFD_Pos;

    /* Configure the GPA0 - GPA3 ADC analog input pins */
    SYS->GPA_MFP &= ~(SYS_GPA_MFP_PA0_Msk | SYS_GPA_MFP_PA1_Msk | SYS_GPA_MFP_PA2_Msk | SYS_GPA_MFP_PA3_Msk) ;
    SYS->GPA_MFP |= SYS_GPA_MFP_PA0_ADC0 | SYS_GPA_MFP_PA1_ADC1 | SYS_GPA_MFP_PA2_ADC2 | SYS_GPA_MFP_PA3_ADC3 ;

    /* Configure the PA12 as PWM0 output pin */
    SYS->GPA_MFP &= ~(SYS_GPA_MFP_PA12_Msk);
    SYS->GPA_MFP |= SYS_GPA_MFP_PA12_PWM0_CH0;


}

/*---------------------------------------------------------------------------------------------------------*/
/* Init UART                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init()
{

    /* Reset UART IP */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_UART0_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART0_RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
    UART0->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: ADC_PWMTrigTest_SingleOpMode                                                                  */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Returns:                                                                                                */
/*   None.                                                                                                 */
/*                                                                                                         */
/* Description:                                                                                            */
/*   ADC hardware trigger test.                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void ADC_PWMTrigTest_SingleOpMode()
{
    uint32_t u32TimeOutCnt;

    printf("\n<<< PWM trigger test (Single mode) >>>\n");

    /* PWM trigger; ADC single operation mode; single-end input; enable the ADC converter. */
    ADC->ADCR = (ADC_ADCR_TRGS_PWM | ADC_ADCR_TRGEN_ENABLE | ADC_ADCR_ADMD_SINGLE | ADC_ADCR_DIFFEN_SINGLE_END | ADC_ADCR_ADEN_CONVERTER_ENABLE);
    /* Enable analog input channel 2 */
    ADC->ADCHER |= ((ADC->ADCHER & ~ADC_ADCHER_CHEN_Msk) | (1 << 2));
    /* Clear the A/D interrupt flag for safe */
    ADC->ADSR = ADC_ADSR_ADF_Msk;


    /* Center-aligned type */
    PWM_SET_ALIGNED_TYPE(PWM0, PWM_CH_0_MASK, PWM_CENTER_ALIGNED);
    /* Clock prescaler */
    PWM_SET_PRESCALER(PWM0, 0, 1);
    /* PWM counter value */ /* PWM frequency = PWM clock source/(clock prescaler setting + 1)/(CNR+1) */
    PWM_SET_CNR(PWM0, 0, 5);
    /* PWM compare value */
    PWM_SET_CMR(PWM0, 0, 1);
    /* Enable PWM0 to trigger ADC */
    PWM0->ADCTS0 = PWM_TRIGGER_ADC_EVEN_PERIOD_POINT | PWM_ADCTS0_TRGEN0_Msk;
    /* PWM0 pin output enabled. PWM frequency 1MHz, duty 30%. */
    PWM_SET_OUTPUT_LEVEL(PWM0, PWM_CH_0_MASK, PWM_OUTPUT_HIGH, PWM_OUTPUT_NOTHING, PWM_OUTPUT_LOW, PWM_OUTPUT_NOTHING);

    PWM0->POEN = PWM_POEN_POEN0_Msk;
    /* Enable the PWM0 counter */
    PWM0->CNTEN |= PWM_CNTEN_CNTEN0_Msk;

    /* wait for one cycle */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while((PWM0->INTSTS0 & PWM_INTSTS0_PIF0_Msk) == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for PWM period interrupt time-out!\n");
            return;
        }
    }

    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while((PWM0->INTSTS0 & PWM_INTSTS0_ZIF0_Msk) == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for PWM zero interrupt time-out!\n");
            return;
        }
    }

    PWM0->INTSTS0 = PWM_INTSTS0_PIF0_Msk | PWM_INTSTS0_ZIF0_Msk;

    /* Disable the PWM0 counter */
    PWM0->CNTEN = 0;

    /* Wait conversion done */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(!((ADC->ADSR & ADC_ADSR_ADF_Msk) >> ADC_ADSR_ADF_Pos))
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for ADC conversion done time-out!\n");
            return;
        }
    }

    /* Clear the ADC interrupt flag */
    ADC->ADSR = ADC_ADSR_ADF_Msk;

    printf("Channel 2: 0x%X\n", ADC->ADDR[2] & 0xFFF);

    /* Disable ADC */
    ADC->ADCR = 0;
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

    /* Init UART0 for printf */
    UART0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* ADC hardware trigger test */
    ADC_PWMTrigTest_SingleOpMode();

    /* Reset ADC module */
    SYS->IPRSTC2 |= (1 << SYS_IPRSTC2_ADC_RST_Pos) ;
    SYS->IPRSTC2 &= ~(1 << (SYS_IPRSTC2_ADC_RST_Pos)) ;

    /* Disable ADC IP clock */
    CLK->APBCLK &= ~CLK_APBCLK_ADC_EN_Msk;

    /* Disable External Interrupt */
    NVIC_DisableIRQ(ADC_IRQn);

    printf("Exit ADC sample code\n");

    while(1);

}

