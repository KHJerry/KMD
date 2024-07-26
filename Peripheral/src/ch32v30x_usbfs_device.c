/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32v30x_usbfs_device.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2022/08/20
* Description        : This file provides all the USBOTG firmware functions.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#include "ch32v30x_usbfs_device.h"
#include "ch32v30x_rcc.h"

/*******************************************************************************/
/* Variable Definition */
/* Global */
const    uint8_t  *pUSBFS_Descr;

/* Setup Request */
volatile uint8_t  USBFS_SetupReqCode;
volatile uint8_t  USBFS_SetupReqType;
volatile uint16_t USBFS_SetupReqValue;
volatile uint16_t USBFS_SetupReqIndex;
volatile uint16_t USBFS_SetupReqLen;

/* USB Device Status */
volatile uint8_t  USBFS_DevConfig;
volatile uint8_t  USBFS_DevAddr;
volatile uint8_t  USBFS_DevSleepStatus;
volatile uint8_t  USBFS_DevEnumStatus;

/* Endpoint Buffer */
__attribute__ ((aligned(4))) uint8_t USBFS_EP0_Buf[ DEF_USBD_UEP0_SIZE ];
__attribute__ ((aligned(4))) uint8_t USBFS_EP1_Buf[ DEF_USBD_ENDP1_SIZE ];
__attribute__ ((aligned(4))) uint8_t USBFS_EP2_Buf[ DEF_USBD_ENDP2_SIZE ];
__attribute__ ((aligned(4))) uint8_t USBFS_EP3_Buf[ DEF_USBD_ENDP3_SIZE ];

/* USB IN Endpoint Busy Flag */
volatile uint8_t  USBFS_Endp_Busy[ DEF_UEP_NUM ];

/******************************************************************************/
/* Interrupt Service Routine Declaration*/


/*********************************************************************
 * @fn      USBFS_RCC_Init
 *
 * @brief   Initializes the usbotg clock configuration.
 *
 * @return  none
 */
void USBFS_RCC_Init(void)
{
#ifdef CH32V30x_D8C
    RCC_USBCLK48MConfig( RCC_USBCLK48MCLKSource_USBPHY );
    RCC_USBHSPLLCLKConfig( RCC_HSBHSPLLCLKSource_HSE );
    RCC_USBHSConfig( RCC_USBPLL_Div2 );
    RCC_USBHSPLLCKREFCLKConfig( RCC_USBHSPLLCKREFCLK_4M );
    RCC_USBHSPHYPLLALIVEcmd( ENABLE );
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_USBHS, ENABLE );
#else
    if( SystemCoreClock == 144000000 )
    {
        RCC_OTGFSCLKConfig( RCC_OTGFSCLKSource_PLLCLK_Div3 );
    }
    else if( SystemCoreClock == 96000000 ) 
    {
        RCC_OTGFSCLKConfig( RCC_OTGFSCLKSource_PLLCLK_Div2 );
    }
    else if( SystemCoreClock == 48000000 ) 
    {
        RCC_OTGFSCLKConfig( RCC_OTGFSCLKSource_PLLCLK_Div1 );
    }
#endif
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_OTG_FS, ENABLE );
}


/*********************************************************************
 * @fn      USBFS_Device_Endp_Init
 *
 * @brief   Initializes USB device endpoints.
 *
 * @return  none
 ********************************************************************/
void USBFS_Device_Endp_Init( void )
{

    USBOTG_FS->UEP4_1_MOD = USBFS_UEP1_TX_EN;
    USBOTG_FS->UEP2_3_MOD = USBFS_UEP2_RX_EN|USBFS_UEP3_TX_EN;

    USBOTG_FS->UEP0_DMA = (uint32_t)USBFS_EP0_Buf;

    USBOTG_FS->UEP1_DMA = (uint32_t)USBFS_EP1_Buf;
    USBOTG_FS->UEP2_DMA = (uint32_t)USBFS_EP2_Buf;
    USBOTG_FS->UEP3_DMA = (uint32_t)USBFS_EP3_Buf;

    USBOTG_FS->UEP0_RX_CTRL = USBFS_UEP_R_RES_ACK;
    USBOTG_FS->UEP2_RX_CTRL = USBFS_UEP_R_RES_ACK;

    USBOTG_FS->UEP1_TX_LEN = 0;
    USBOTG_FS->UEP3_TX_LEN = 0;

    USBOTG_FS->UEP0_TX_CTRL = USBFS_UEP_T_RES_NAK;
    USBOTG_FS->UEP1_TX_CTRL = USBFS_UEP_T_RES_NAK;
    USBOTG_FS->UEP3_TX_CTRL = USBFS_UEP_T_RES_NAK;

    /* Clear End-points Busy Status */
    for(uint8_t i=0; i<DEF_UEP_NUM; i++ )
    {
        USBFS_Endp_Busy[i] = 0;
    }
}

/*********************************************************************
 * @fn      USBFS_Device_Init
 *
 * @brief   Initializes USB device.
 *
 * @return  none
 *********************************************************************/
void USBFS_Device_Init( FunctionalState sta )
{
    if( sta )
    {
        USBOTG_H_FS->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL;
        Delay_Us( 10 );
        USBOTG_H_FS->BASE_CTRL = 0x00;
        USBOTG_FS->INT_EN    = USBFS_UIE_SUSPEND | USBFS_UIE_BUS_RST | USBFS_UIE_TRANSFER;
        USBOTG_FS->BASE_CTRL = USBFS_UC_DEV_PU_EN | USBFS_UC_INT_BUSY | USBFS_UC_DMA_EN;
        USBFS_Device_Endp_Init( );
        USBOTG_FS->UDEV_CTRL = USBFS_UD_PD_DIS | USBFS_UD_PORT_EN;
        NVIC_EnableIRQ( OTG_FS_IRQn );
    }
    else
    {
        USBOTG_H_FS->BASE_CTRL = USBFS_UC_RESET_SIE | USBFS_UC_CLR_ALL;
        Delay_Us( 10 );
        USBOTG_FS->BASE_CTRL = 0x00;
        NVIC_DisableIRQ( OTG_FS_IRQn );
    }
}

/*********************************************************************
 * @fn      USBFS_Endp_DataUp
 *
 * @brief   USBFS device data upload
 *
 * @return  none
 *********************************************************************/
uint8_t USBFS_Endp_DataUp(uint8_t endp, uint8_t *pbuf, uint16_t len, uint8_t mod)
{
    uint8_t endp_mode;
    uint8_t buf_load_offset;

    /* DMA config, endp_ctrl config, endp_len config */
    if( (endp>=DEF_UEP1) && (endp<=DEF_UEP7) )
    {
        if( USBFS_Endp_Busy[ endp ] == 0 )
        {
            if( (endp == DEF_UEP1) || (endp == DEF_UEP4) )
            {
                /* endp1/endp4 */
                endp_mode = USBFSD_UEP_MOD(0);
                if( endp == DEF_UEP1 )
                {
                    endp_mode = (uint8_t)(endp_mode>>4);
                }
            }
            else if( (endp == DEF_UEP2) || (endp == DEF_UEP3) )
            {
                /* endp2/endp3 */
                endp_mode = USBFSD_UEP_MOD(1);
                if( endp == DEF_UEP3 )
                {
                    endp_mode = (uint8_t)(endp_mode>>4);
                }
            }
            else if( (endp == DEF_UEP5) || (endp == DEF_UEP6) )
            {
                /* endp5/endp6 */
                endp_mode = USBFSD_UEP_MOD(2);
                if( endp == DEF_UEP6 )
                {
                    endp_mode = (uint8_t)(endp_mode>>4);
                }
            }
            else
            {
                /* endp7 */
                endp_mode = USBFSD_UEP_MOD(3);
            }

            if( endp_mode & USBFSD_UEP_TX_EN )
            {
                if( endp_mode & USBFSD_UEP_RX_EN )
                {
                    buf_load_offset = 64;
                }
                else
                {
                    buf_load_offset = 0;
                }

                if( buf_load_offset == 0 )
                {
                    if( mod == DEF_UEP_DMA_LOAD )
                    {
                        /* DMA mode */
                        USBFSD_UEP_DMA(endp) = (uint16_t)(uint32_t)pbuf;
                    }
                    else
                    {
                        /* copy mode */
                        memcpy( USBFSD_UEP_BUF(endp), pbuf, len );
                    }
                }
                else
                {
                    memcpy( USBFSD_UEP_BUF(endp)+buf_load_offset, pbuf, len );
                }
                /* Set end-point busy */
                USBFS_Endp_Busy[ endp ] = 0x01;
                /* tx length */
                USBFSD_UEP_TLEN(endp) = len;
                /* response ack */
                USBFSD_UEP_TX_CTRL(endp) = (USBFSD_UEP_TX_CTRL(endp) & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_ACK;
            }
        }
        else
        {
            return 1;
        }
    }
    else
    {
        return 1;
    }
    return 0;
}


/*********************************************************************
 * @fn      USBFS_Send_Resume
 *
 * @brief   USBFS device sends wake-up signal to host
 *
 * @return  none
 */
void USBFS_Send_Resume(void)
{

}

