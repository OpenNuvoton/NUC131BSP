/****************************************************************************
 * @file     main.c
 * @version  V2.0
 * $Revision: 8 $
 * $Date: 15/01/16 1:45p $
 * @brief
 *           Implement receive message in Basic mode.
 *           This sample code needs to work with CAN_BasicMode_Transmit.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC131.h"


#define PLL_CLOCK       48000000

/*---------------------------------------------------------------------------*/
/*  Function Declare                                                         */
/*---------------------------------------------------------------------------*/
extern char GetChar(void);
void CAN_ShowMsg(STR_CANMSG_T* Msg);

/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
/* Declare a CAN message structure */
STR_CANMSG_T rrMsg;

/*---------------------------------------------------------------------------------------------------------*/
/* CAN0 interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void CAN0_IRQHandler(void)
{
    uint32_t u8IIDRstatus;

    u8IIDRstatus = CAN0->IIDR;

    if(u8IIDRstatus == 0x00008000)        /* Check Status Interrupt Flag (Error status Int and Status change Int) */
    {
        /**************************/
        /* Status Change interrupt*/
        /**************************/
        if(CAN0->STATUS & CAN_STATUS_RXOK_Msk)
        {
            CAN0->STATUS &= ~CAN_STATUS_RXOK_Msk;   /* Clear RxOK status*/
        }

        if(CAN0->STATUS & CAN_STATUS_TXOK_Msk)
        {
            CAN0->STATUS &= ~CAN_STATUS_TXOK_Msk;    /* Clear TxOK status*/
        }

        /**************************/
        /* Error Status interrupt */
        /**************************/
        if(CAN0->STATUS & CAN_STATUS_BOFF_Msk)
        {
            printf("BOFF INT\n") ;
        }
        else if(CAN0->STATUS & CAN_STATUS_EWARN_Msk)
        {
            printf("EWARN INT\n") ;
        }
        else if((CAN0->ERR & CAN_ERR_TEC_Msk) != 0)
        {
            printf("Transmit error!\n") ;
        }
        else if((CAN0->ERR & CAN_ERR_REC_Msk) != 0)
        {
            printf("Receive error!\n") ;
        }

    }
    else if((u8IIDRstatus >= 0x1) || (u8IIDRstatus <= 0x20))
    {
        CAN_CLR_INT_PENDING_BIT(CAN0, (u8IIDRstatus - 1)); /* Clear Interrupt Pending */
    }
    else if(CAN0->WU_STATUS == 1)
    {
        printf("Wake up\n");

        CAN0->WU_STATUS = 0;    /* Write '0' to clear */
    }

}

/*---------------------------------------------------------------------------------------------------------*/
/* Reset message interface parameters                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_ResetIF(CAN_T *tCAN, uint8_t u8IF_Num)
{
    if(u8IF_Num > 1)
        return;
    tCAN->IF[u8IF_Num].CREQ     = 0x0;          // set bit15 for sending
    tCAN->IF[u8IF_Num].CMASK    = 0x0;
    tCAN->IF[u8IF_Num].MASK1    = 0x0;          // useless in basic mode
    tCAN->IF[u8IF_Num].MASK2    = 0x0;          // useless in basic mode
    tCAN->IF[u8IF_Num].ARB1     = 0x0;          // ID15~0
    tCAN->IF[u8IF_Num].ARB2     = 0x0;          // MsgVal, eXt, xmt, ID28~16
    tCAN->IF[u8IF_Num].MCON     = 0x0;              // DLC
    tCAN->IF[u8IF_Num].DAT_A1   = 0x0;          // data0,1
    tCAN->IF[u8IF_Num].DAT_A2   = 0x0;          // data2,3
    tCAN->IF[u8IF_Num].DAT_B1   = 0x0;          // data4,5
    tCAN->IF[u8IF_Num].DAT_B2   = 0x0;          // data6,7
}

/*---------------------------------------------------------------------------*/
/*  Show Message Function                                                    */
/*---------------------------------------------------------------------------*/
void CAN_ShowMsg(STR_CANMSG_T* Msg)
{
    uint8_t i;

    /* Show the message information */
    printf("Read ID=0x%X, Type=%s, DLC=%d, Data=", Msg->Id, Msg->IdType ? "EXT" : "STD", Msg->DLC);
    for(i = 0; i < Msg->DLC; i++)
        printf("%X,", Msg->Data[i]);
    printf("\n\n");
}

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

    /* Enable CAN module clock */
    CLK_EnableModuleClock(CAN0_MODULE);
    //CLK_EnableModuleClock(CAN1_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_PLL, CLK_CLKDIV_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;

    /* Set PD multi-function pins for CANTX0, CANRX0 */
    SYS->GPD_MFP &= ~(SYS_GPD_MFP_PD6_Msk | SYS_GPD_MFP_PD7_Msk);
    SYS->GPD_MFP |= SYS_GPD_MFP_PD6_CAN0_RXD | SYS_GPD_MFP_PD7_CAN0_TXD;

}

/*---------------------------------------------------------------------------------------------------------*/
/* Init UART                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_Init()
{
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Disable CAN Clock and Reset it                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_STOP(void)
{
    /* Disable CAN0 Clock and Reset it */
    SYS_ResetModule(CAN0_RST);
    CLK_DisableModuleClock(CAN0_MODULE);
}

/*----------------------------------------------------------------------------*/
/*  Some description about how to create test environment                     */
/*----------------------------------------------------------------------------*/
void Note_Configure()
{
    printf("\n\n");
    printf("+--------------------------------------------------------------------------+\n");
    printf("|  About CAN sample code configure                                         |\n");
    printf("+--------------------------------------------------------------------------+\n");
    printf("|   The sample code provide a simple sample code for you study CAN         |\n");
    printf("|   Before execute it, please check description as below                   |\n");
    printf("|                                                                          |\n");
    printf("|   1.CAN_TX and CAN_RX should be connected to your CAN transceiver        |\n");
    printf("|   2.Using two module board and connect to the same CAN BUS               |\n");
    printf("|   3.Check the terminal resistor of bus is connected                      |\n");
    printf("|   4.Using UART0 as print message port                                    |\n");
    printf("|                                                                          |\n");
    printf("|  |--------|       |-----------|   CANBUS  |-----------|       |--------| |\n");
    printf("|  |        |------>|           |<--------->|           |<------|        | |\n");
    printf("|  |        |CAN_TX |    CAN    |   CAN_H   |   CAN     |CAN_TX |        | |\n");
    printf("|  | NUC131 |       |Transceiver|           |Transceiver|       | NUC131 | |\n");
    printf("|  |        |<------|           |<--------->|           |------>|        | |\n");
    printf("|  |        |CAN_RX |           |   CAN_L   |           |CAN_RX |        | |\n");
    printf("|  |--------|       |-----------|           |-----------|       |--------| |\n");
    printf("|  |                                                            |          |\n");
    printf("|  |                                                            |          |\n");
    printf("|  V                                                            V          |\n");
    printf("| UART0                                                         UART0      |\n");
    printf("|(print message)                                          (print message)  |\n");
    printf("+--------------------------------------------------------------------------+\n");
}

/*----------------------------------------------------------------------------*/
/*  Check the real baud-rate                                                  */
/*----------------------------------------------------------------------------*/
void BaudRateCheck(uint32_t u32BaudRate, uint32_t u32RealBaudRate)
{
    /* Get Core Clock Frequency */
    SystemCoreClockUpdate();

    if(u32BaudRate != u32RealBaudRate)
    {
        printf("Set CAN baud-rate is fail\n");
        printf("Real baud-rate value(bps): %d\n", u32RealBaudRate);
        printf("CAN baud-rate calculation equation as below:\n");
        printf("CAN baud-rate(bps) = Fin/(BPR+1)*(Tseg1+Tseg2+3)\n");
        printf("where: Fin: System clock freq.(Hz)\n");
        printf("       BRP: The baud rate prescale. It is composed of BRP (CAN_BTIME[5:0]) and BRPE (CAN_BRPE[3:0]).\n");
        printf("       Tseg1: Time Segment before the sample point. You can set tseg1 (CAN_BTIME[11:8]).\n");
        printf("       Tseg2: Time Segment after the sample point. You can set tseg2 (CAN_BTIME[14:12]).\n");

        if(SystemCoreClock % u32BaudRate != 0)
            printf("\nThe BPR does not calculate, the Fin must be a multiple of the CAN baud-rate.\n");

        else
            printf("\nThe BPR does not calculate, the (Fin/(CAN baud-rate)) must be a multiple of the (Tseg1+Tseg1+3).\n");
    }
    else
        printf("\nReal baud-rate value(bps): %d\n", u32RealBaudRate);
}

/*----------------------------------------------------------------------------*/
/*  Set the CAN speed                                                         */
/*----------------------------------------------------------------------------*/
void SelectCANSpeed(CAN_T  *tCAN)
{
    uint32_t unItem, BaudRate = 0, RealBaudRate = 0;

    printf("Please select CAN speed you desired\n");
    printf("[0] 1000Kbps\n");
    printf("[1]  800Kbps\n");
    printf("[2]  500Kbps\n");
    printf("[3]  250Kbps\n");
    printf("[4]  125Kbps\n");
    printf("[5]  100Kbps\n");
    printf("[6]   50Kbps\n");

    unItem = GetChar();
    printf("%c\n", unItem);

    switch(unItem)
    {
        case '0':
            BaudRate = 1000000;
            RealBaudRate = CAN_Open(tCAN,  BaudRate, CAN_BASIC_MODE);//Set target baud-rate and operation mode.
            break;

        case '1':
            BaudRate = 800000;
            RealBaudRate = CAN_Open(tCAN,  BaudRate, CAN_BASIC_MODE);
            break;

        case '2':
            BaudRate = 500000;
            RealBaudRate = CAN_Open(tCAN,  BaudRate, CAN_BASIC_MODE);
            break;

        case '3':
            BaudRate = 250000;
            RealBaudRate = CAN_Open(tCAN,  BaudRate, CAN_BASIC_MODE);
            break;

        case '4':
            BaudRate = 125000;
            RealBaudRate = CAN_Open(tCAN,  BaudRate, CAN_BASIC_MODE);
            break;

        case '5':
            BaudRate = 100000;
            RealBaudRate = CAN_Open(tCAN,  BaudRate, CAN_BASIC_MODE);
            break;

        case '6':
            BaudRate = 50000;
            RealBaudRate = CAN_Open(tCAN,  BaudRate, CAN_BASIC_MODE);
            break;
    }

    /* Check the real baud-rate is OK */
    BaudRateCheck(BaudRate, RealBaudRate);

}

/*----------------------------------------------------------------------------*/
/*  Test Menu                                                                 */
/*----------------------------------------------------------------------------*/
void TestItem(void)
{
    printf("\n");
    printf("+------------------------------------------------------------------ +\n");
    printf("|  Nuvoton CAN BUS DRIVER DEMO                                      |\n");
    printf("+-------------------------------------------------------------------+\n");
    printf("|                                                                   |\n");
    printf("|      Receive a message by basic mode                              |\n");
    printf("|                                                                   |\n");
    printf("+-------------------------------------------------------------------+\n");
}

/*----------------------------------------------------------------------------*/
/*  Read Rx Msg by Basic Mode Function (Without Message RAM)                  */
/*----------------------------------------------------------------------------*/
void Test_BasicMode_Rx(CAN_T *tCAN)
{

    STR_CANMSG_T rMsg[5];
    int32_t i;

    /* Enable CAN interrupt */
    CAN_EnableInt(tCAN, CAN_CON_IE_Msk | CAN_CON_SIE_Msk);

    /* Set Interrupt Priority */
    NVIC_SetPriority(CAN0_IRQn, (1 << __NVIC_PRIO_BITS) - 2);

    /* Enable External Interrupt */
    NVIC_EnableIRQ(CAN0_IRQn);

    /*  Wait status flag changed and with read IF2 */
    printf("\b Total 40 bytes data(using 5 frames)will be receive by CAN0 from CAN BUS\n");

    for(i = 0; i < 5; i++)
    {
        CAN_ResetIF(tCAN, 1);
        tCAN->IF[1].MCON = 0;
        while(CAN_Receive(tCAN, 0, &rMsg[i]) == FALSE);
    }
    for(i = 0; i < 5; i++)
        CAN_ShowMsg(&rMsg[i]);
    printf("CAN TEST END\n");
    return;
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    CAN_T *tCAN;
    tCAN = (CAN_T *) CAN0;

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

    /* Some description about how to create test environment */
    Note_Configure();

    /* Configuring the Bit Timing */
    SelectCANSpeed(tCAN);

    /* Test Menu */
    TestItem();

    printf("Receive a message by basic mode\n\n");

    /* Read Rx Msg by Basic Mode Function (Without Message RAM) */
    Test_BasicMode_Rx(tCAN);

    /* Disable CAN */
    CAN_Close(tCAN);

    /* Disable CAN Clock and Reset it */
    CAN_STOP();

    while(1);

}



