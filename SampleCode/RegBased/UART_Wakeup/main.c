/****************************************************************************
 * @file     main.c
 * @version  V2.00
 * $Revision: 2 $
 * $Date: 15/07/10 11:07a $
 * @brief    Show how to wake up system from Power-down mode by UART interrupt.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC131.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void UART_DataWakeUp(void);
void UART_CTSWakeUp(void);
void UART_PowerDown_TestItem(void);
void UART_PowerDownWakeUpTest(void);


/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    uint32_t u32TimeOutCnt;

    /* Check if all the debug messages are finished */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    UART_WAIT_TX_EMPTY(UART0)
        if(--u32TimeOutCnt == 0) break;

    /* Set the processor is deep sleep as its low power mode */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    /* Set system Power-down enabled and Power-down entry condition */
    CLK->PWRCON |= (CLK_PWRCON_PWR_DOWN_EN_Msk | CLK_PWRCON_PD_WAIT_CPU_Msk);    

    /* Chip enter Power-down mode after CPU run WFI instruction */
    __WFI();
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
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLK_S_Msk)) | CLK_CLKSEL0_HCLK_S_HIRC;
    CLK->CLKDIV = (CLK->CLKDIV & (~CLK_CLKDIV_HCLK_N_Msk)) | CLK_CLKDIV_HCLK(1);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Enable UART module clock */
    CLK->APBCLK |= (CLK_APBCLK_UART0_EN_Msk | CLK_APBCLK_UART1_EN_Msk);

    /* Select UART module clock source */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART_S_Msk)) | CLK_CLKSEL1_UART_S_HIRC;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    /* Set GPB multi-function pins for UART1 RXD, TXD and nCTS */

    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk |
                      SYS_GPB_MFP_PB4_Msk | SYS_GPB_MFP_PB5_Msk | SYS_GPB_MFP_PB7_Msk);

    SYS->GPB_MFP |= (SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD |
                     SYS_GPB_MFP_PB4_UART1_RXD | SYS_GPB_MFP_PB5_UART1_TXD | SYS_GPB_MFP_PB7_UART1_nCTS);

}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_UART0_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART0_RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    UART0->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

void UART1_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART1 */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_UART1_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART1_RST_Msk;

    /* Configure UART1 and set UART1 Baudrate */
    UART1->BAUD = UART_BAUD_MODE0 | UART_BAUD_MODE0_DIVIDER(__HIRC, 110);
    UART1->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
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

    /* Init UART1 for testing */
    UART1_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("\n\nUART Sample Program.\n");

    /* UART wake-up sample function */
    UART_PowerDownWakeUpTest();

    printf("\nUART Sample Program End.\n");

    while(1);

}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 1 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_IRQHandler(void)
{
    uint32_t u32IntSts = UART1->ISR;

    if(u32IntSts & UART_ISR_WKIF_Msk)   /* UART wake-up interrupt flag */
    {
        if(u32IntSts & UART_ISR_DATWKIF_Msk)    /* UART data wake-up interrupt flag */
        {
            printf("UART data wake-up interrupt happen.\n");
            UART1->ISR = UART_ISR_DATWKIF_Msk;  /* Clear UART data wake-up interrupt flag */
        }

        if(u32IntSts & UART_ISR_CTSWKIF_Msk)    /* UART CTS wake-up interrupt flag */
        {
            printf("UART nCTS wake-up interrupt happen.\n");
            UART1->ISR = UART_ISR_CTSWKIF_Msk;  /* Clear UART CTS wake-up interrupt flag */
        }
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART nCTS Wake-up Function                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void UART_CTSWakeUp(void)
{

    /* Clear MODEM interrupt before CTS wake-up interrupt */
    UART1->MSR |= UART_MSR_DCTSF_Msk;

    /* Enable UART CTS wake-up interrupt */
    UART1->IER |= UART_IER_WKCTSIEN_Msk;
    NVIC_EnableIRQ(UART1_IRQn);      

    printf("System enter to Power-down mode.\n");
    printf("Toggle nCTS of UART1 to wake-up system.\n");

    /* Unlock protected registers before entering Power-down mode */
    SYS_UnlockReg();

    /* Enter to Power-down mode */
    PowerDownFunction();

    /* Lock protected registers after entering Power-down mode */
    SYS_LockReg();

    /* Clear MODEM interrupt after CTS wake-up interrupt */
    UART1->MSR |= UART_MSR_DCTSF_Msk;

    /* Disable UART CTS Wake-up Interrupt */
    UART1->IER &= ~UART_IER_WKCTSIEN_Msk;
    NVIC_DisableIRQ(UART1_IRQn);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Data Wake-up Function                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void UART_DataWakeUp(void)
{
    uint32_t u32TimeOutCnt;

    /* Enable UART data wake-up interrupt */
    UART1->IER |= UART_IER_WKDATIEN_Msk;
    NVIC_EnableIRQ(UART1_IRQn);

    printf("System enter to Power-down mode.\n");
    printf("Send data with baud rate 110bps to UART1 to wake-up system.\n");

    /* Unlock protected registers before entering Power-down mode */
    SYS_UnlockReg();

    /* Enter to Power-down mode */
    PowerDownFunction();

    /* Lock protected registers after entering Power-down mode */
    SYS_LockReg();

    /* Wait to receive wake-up data */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(!UART_IS_RX_READY(UART1))
        if(--u32TimeOutCnt == 0) break;

    if(u32TimeOutCnt == 0)
        printf("Wait for receive wake-up data time-out!\n");
    else
        printf("The first wake-up data is 0x%x.\n", UART_READ(UART1));

    /* Disable UART Data Wake-up Interrupt */
    UART1->IER &= ~UART_IER_WKDATIEN_Msk;
    NVIC_DisableIRQ(UART1_IRQn);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Power-down and Wake-up Menu                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void UART_PowerDown_TestItem(void)
{
    printf("\n\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  UART Power-down and wake-up test                         |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| [1] Data wake-up test                                     |\n");
    printf("| [2] nCTS wake-up test                                     |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| Quit                                              - [ESC] |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("Please Select key (1~2): ");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Power-down and Wake-up Test Function                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void UART_PowerDownWakeUpTest(void)
{
    uint32_t u32Item;

    UART_PowerDown_TestItem();
    u32Item = getchar();
    printf("%c\n\n", u32Item);
    switch(u32Item)
    {
        case '1':
            UART_DataWakeUp();
            break;
        case '2':
            UART_CTSWakeUp();
            break;
        default:
            break;
    }
}
