/********************************** (C) COPYRIGHT *******************************
* File Name          : Interrupt.c
* Author             : HK
* Version            : V1.0.0
* Date               : 2023/01/07
* Description        : Interrupt Service Routines.
*********************************************************************************/

#include "Interrupt.h"
#include "usb_desc.h"
#include "ch32v30x_usbfs_device.h"
#include "ch32v30x_misc.h"
#include "HardWare.h"


void NMI_Handler(void)               __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void)         __attribute__((interrupt("WCH-Interrupt-fast")));
void OTG_FS_IRQHandler(void)         __attribute__((interrupt("WCH-Interrupt-fast")));


/*********************************************************************
 * @fn      NMI_Handler
 *
 * @brief   This function handles NMI exception.
 *
 * @return  none
 *********************************************************************/
void NMI_Handler(void)
{

}

/*********************************************************************
 * @fn      HardFault_Handler
 *
 * @brief   This function handles Hard Fault exception.
 *
 * @return  none
 ********************************************************************/
void HardFault_Handler(void)
{
  while (1)
  {
  }
}
/*********************************************************************
 * @fn
 *
 * @brief
 *
 * @return
 ********************************************************************/
void Interrupt_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);             //中断优先级分组配置

    //ADC注入中断配置
    NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;                             //中断通道ADC_IRQn
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;                  //抢占优先级0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                         //从优先级1
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                            //中断使能
    NVIC_Init(&NVIC_InitStructure);                               //配置寄存器实体

    //Timer1中断配置
//    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;                        //中断通道TIM1_UP_IRQn
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;                 //抢占优先级0
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;                         //从优先级0
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                           //中断使能
//    NVIC_Init(&NVIC_InitStructure);                              //配置寄存器实体

    //Timer4中断配置
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;                        //中断通道TIM4_IRQn
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;                 //抢占优先级0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;                         //从优先级0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                           //中断使能
    NVIC_Init(&NVIC_InitStructure);                              //配置寄存器实体

//    //Timer8中断配置
//    NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_IRQn;                           //中断通道TIM8_UP_IRQn
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;                 //抢占优先级0
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                        //从优先级1
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                           //中断使能
//    NVIC_Init(&NVIC_InitStructure);                              //配置寄存器实体

    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*********************************************************************
 * @fn
 *
 * @brief
 *
 * @return
 ********************************************************************/
void OTG_FS_IRQHandler(void)
{
    uint8_t  intflag, intst, errflag;
    uint16_t len;

    intflag = USBOTG_FS->INT_FG;
    intst   = USBOTG_FS->INT_ST;

    //表示此次中断是数据传输中断
    if( intflag & USBFS_UIF_TRANSFER )
    {
        //确定中断的类型
        switch (intst & USBFS_UIS_TOKEN_MASK)
        {
            //说明本次中断是数据传输的IN口（单向数据传输从外设到主机）
            case USBFS_UIS_TOKEN_IN:
                switch ( intst & (USBFS_UIS_TOKEN_MASK | USBFS_UIS_ENDP_MASK ))
                {
                    //针对不同的中断类型，再次进行细分，确定中断来源。
                    //对于IN口中断，检测是否为端点0（即默认控制端点）的接收中断。
                    case USBFS_UIS_TOKEN_IN | DEF_UEP0:
                        if( USBFS_SetupReqLen == 0 )
                        {
                            //如果收到的数据长度为0，则发送ACK应答信号回复主机。
                            USBOTG_FS->UEP0_RX_CTRL = USBFS_UEP_R_TOG | USBFS_UEP_R_RES_ACK;
                        }

                        if ( ( USBFS_SetupReqType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
                        {
                            /* Non-standard request endpoint 0 Data upload */

                        }
                        else
                        {
                            //根据请求的类型（SETUP、OUT、IN）来判断执行哪种操作。
                            switch( USBFS_SetupReqCode )
                            {
                                //如果是获取设备描述符，则从全局定义的描述符变量中返回一段数据，将数据长度保存到len变量中。
                                case USB_GET_DESCRIPTOR:
                                    len = USBFS_SetupReqLen >= DEF_USBD_UEP0_SIZE ? DEF_USBD_UEP0_SIZE : USBFS_SetupReqLen; //计算实际需要返回的数据长度，取值范围为1~64字节。
                                    memcpy( USBFS_EP0_Buf, pUSBFS_Descr, len );                                             //将从描述符变量中返回的数据拷贝到端点0缓存中。
                                    USBFS_SetupReqLen -= len;                                                               //更新待返回的数据长度，即减去已经返回的数据长度。
                                    pUSBFS_Descr += len;                                                                    //将描述符变量的指针向后移动，指向下一个待返回的数据。
                                    USBOTG_FS->UEP0_TX_LEN   = len;                                                         //设置端点0的发送数据长度寄存器的值。
                                    USBOTG_FS->UEP0_TX_CTRL ^= USBFS_UEP_T_TOG;                                             //翻转端点0的发送数据的数据包序号，用于区分不同的数据包。
                                    break;
                                    //如果是设置设备地址，则从芯片设备寄存器中获取当前设备地址，并将它与新的设备地址进行合并。
                                case USB_SET_ADDRESS:
                                    USBOTG_FS->DEV_ADDR = (USBOTG_FS->DEV_ADDR & USBFS_UDA_GP_BIT) | USBFS_DevAddr;         //将新的设备地址合并到设备地址寄存器中，并将低两位清零
                                    break;

                                default:
                                    break;
                            }
                        }
                        break;

                        /* end-point 1 data in interrupt */
                    case ( USBFS_UIS_TOKEN_IN | DEF_UEP1 ):
                        USBOTG_FS->UEP1_TX_CTRL ^= USBFS_UEP_T_TOG;
                        USBOTG_FS->UEP1_TX_CTRL  = (USBOTG_FS->UEP1_TX_CTRL & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
                        USBFS_Endp_Busy[ DEF_UEP1 ] = 0;
                        break;

                        /* end-point 3 data in interrupt */
                    case ( USBFS_UIS_TOKEN_IN | DEF_UEP3 ):
                        USBOTG_FS->UEP3_TX_CTRL ^= USBFS_UEP_T_TOG;
                        USBOTG_FS->UEP3_TX_CTRL = (USBOTG_FS->UEP3_TX_CTRL & ~USBFS_UEP_T_RES_MASK) | USBFS_UEP_T_RES_NAK;
                        USBFS_Endp_Busy[ DEF_UEP3 ] = 0;
                        break;

                    default :
                        break;
                }
                break;

                //说明本次中断是数据传输的OUT口（单向数据传输从主机到外设）。
            case USBFS_UIS_TOKEN_OUT:
                switch ( intst & ( USBFS_UIS_TOKEN_MASK | USBFS_UIS_ENDP_MASK ) )
                {
                    /* end-point 0 data out interrupt */
                    case USBFS_UIS_TOKEN_OUT | DEF_UEP0:
                        len = USBOTG_FS->RX_LEN;
                        if ( intst & USBFS_UIS_TOG_OK )
                        {
                            if ( ( USBFS_SetupReqType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
                            {
                                /* Non-standard request end-point 0 Data download */
                                USBFS_SetupReqLen = 0;
                                /* Non-standard request end-point 0 Data download */
                                if( USBFS_SetupReqCode == CDC_SET_LINE_CODING )
                                {
                                }
                            }
                            else
                            {
                                /* Standard request end-point 0 Data download */
                                /* Add your code here */
                            }
                            if( USBFS_SetupReqLen == 0 )
                            {
                                USBOTG_FS->UEP0_TX_LEN  = 0;
                                USBOTG_FS->UEP0_TX_CTRL = USBFS_UEP_T_TOG | USBFS_UEP_T_RES_ACK;
                            }
                        }
                        break;

                        /* end-point 1 data out interrupt */
                    case USBFS_UIS_TOKEN_OUT | DEF_UEP2:

                    default:
                        break;
                }
                break;

                //说明本次中断是设置请求。
            case USBFS_UIS_TOKEN_SETUP:
                USBOTG_FS->UEP0_TX_CTRL = USBFS_UEP_T_TOG|USBFS_UEP_T_RES_NAK;
                USBOTG_FS->UEP0_RX_CTRL = USBFS_UEP_R_TOG|USBFS_UEP_R_RES_NAK;
                /* Store All Setup Values */
                USBFS_SetupReqType  = pUSBFS_SetupReqPak->bRequestType;
                USBFS_SetupReqCode  = pUSBFS_SetupReqPak->bRequest;
                USBFS_SetupReqLen   = pUSBFS_SetupReqPak->wLength;
                USBFS_SetupReqValue = pUSBFS_SetupReqPak->wValue;
                USBFS_SetupReqIndex = pUSBFS_SetupReqPak->wIndex;
                len = 0;
                errflag = 0;
                if ( ( USBFS_SetupReqType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
                {
                    /* usb non-standard request processing */
                    if( USBFS_SetupReqType & USB_REQ_TYP_CLASS )
                    {
                        /* Class requests */
                        switch( USBFS_SetupReqCode )
                        {
                            case CDC_GET_LINE_CODING:
                                break;

                            case CDC_SET_LINE_CODING:
                                break;

                            case CDC_SET_LINE_CTLSTE:
                                break;

                            case CDC_SEND_BREAK:
                                break;

                            default:
                                errflag = 0xff;
                                break;
                        }
                    }
                    else if( USBFS_SetupReqType & USB_REQ_TYP_VENDOR )
                    {
                        /* Manufacturer request */
                    }
                    else
                    {
                        errflag = 0xFF;
                    }

                    /* Copy Descriptors to Endp0 DMA buffer */
                    len = (USBFS_SetupReqLen >= DEF_USBD_UEP0_SIZE) ? DEF_USBD_UEP0_SIZE : USBFS_SetupReqLen;
                    memcpy( USBFS_EP0_Buf, pUSBFS_Descr, len );
                    pUSBFS_Descr += len;
                }
                else
                {
                    /* usb standard request processing */
                    switch( USBFS_SetupReqCode )
                    {
                        /* get device/configuration/string/report/... descriptors */
                        case USB_GET_DESCRIPTOR:
                            switch( (uint8_t)( USBFS_SetupReqValue >> 8 ) )
                            {
                                /* get usb device descriptor */
                                case USB_DESCR_TYP_DEVICE:
                                    pUSBFS_Descr = MyDevDescr;
                                    len = DEF_USBD_DEVICE_DESC_LEN;
                                    break;

                                    /* get usb configuration descriptor */
                                case USB_DESCR_TYP_CONFIG:
                                    pUSBFS_Descr = MyCfgDescr;
                                    len = DEF_USBD_CONFIG_DESC_LEN;
                                    break;

                                    /* get usb string descriptor */
                                case USB_DESCR_TYP_STRING:
                                    switch( (uint8_t)( USBFS_SetupReqValue & 0xFF ) )
                                    {
                                        /* Descriptor 0, Language descriptor */
                                        case DEF_STRING_DESC_LANG:
                                            pUSBFS_Descr = MyLangDescr;
                                            len = DEF_USBD_LANG_DESC_LEN;
                                            break;

                                            /* Descriptor 1, Manufacturers String descriptor */
                                        case DEF_STRING_DESC_MANU:
                                            pUSBFS_Descr = MyManuInfo;
                                            len = DEF_USBD_MANU_DESC_LEN;
                                            break;

                                            /* Descriptor 2, Product String descriptor */
                                        case DEF_STRING_DESC_PROD:
                                            pUSBFS_Descr = MyProdInfo;
                                            len = DEF_USBD_PROD_DESC_LEN;
                                            break;

                                            /* Descriptor 3, Serial-number String descriptor */
                                        case DEF_STRING_DESC_SERN:
                                            pUSBFS_Descr = MySerNumInfo;
                                            len = DEF_USBD_SN_DESC_LEN;
                                            break;

                                        default:
                                            errflag = 0xFF;
                                            break;
                                    }
                                    break;

                                default :
                                    errflag = 0xFF;
                                    break;
                            }

                            /* Copy Descriptors to Endp0 DMA buffer */
                            if( USBFS_SetupReqLen>len )
                            {
                                USBFS_SetupReqLen = len;
                            }
                            len = (USBFS_SetupReqLen >= DEF_USBD_UEP0_SIZE) ? DEF_USBD_UEP0_SIZE : USBFS_SetupReqLen;
                            memcpy( USBFS_EP0_Buf, pUSBFS_Descr, len );
                            pUSBFS_Descr += len;
                            break;

                            /* Set usb address */
                        case USB_SET_ADDRESS:
                            USBFS_DevAddr = (uint8_t)( USBFS_SetupReqValue & 0xFF );
                            break;

                            /* Get usb configuration now set */
                        case USB_GET_CONFIGURATION:
                            USBFS_EP0_Buf[0] = USBFS_DevConfig;
                            if ( USBFS_SetupReqLen > 1 )
                            {
                                USBFS_SetupReqLen = 1;
                            }
                            break;

                            /* Set usb configuration to use */
                        case USB_SET_CONFIGURATION:
                            USBFS_DevConfig = (uint8_t)( USBFS_SetupReqValue & 0xFF );
                            USBFS_DevEnumStatus = 0x01;
                            break;

                            /* Clear or disable one usb feature */
                        case USB_CLEAR_FEATURE:
                            if ( ( USBFS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
                            {
                                /* clear one device feature */
                                if( (uint8_t)( USBFS_SetupReqValue & 0xFF ) == USB_REQ_FEAT_REMOTE_WAKEUP )
                                {
                                    /* clear usb sleep status, device not prepare to sleep */
                                    USBFS_DevSleepStatus &= ~0x01;
                                }
                            }
                            else if( ( USBFS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
                            {
                                /* Clear End-point Feature */
                                if( (uint8_t)( USBFS_SetupReqValue & 0xFF ) == USB_REQ_FEAT_ENDP_HALT )
                                {
                                    switch( (uint8_t)( USBFS_SetupReqIndex & 0xFF ) )
                                    {
                                        case ( DEF_UEP_IN | DEF_UEP1 ):
                                            /* Set End-point 1 IN NAK */
                                            USBOTG_FS->UEP1_TX_CTRL = USBFS_UEP_T_RES_NAK;
                                            break;

                                        case ( DEF_UEP_OUT | DEF_UEP2 ):
                                            /* Set End-point 2 OUT ACK */
                                            USBOTG_FS->UEP2_RX_CTRL = USBFS_UEP_R_RES_ACK;
                                            break;

                                        case ( DEF_UEP_IN | DEF_UEP3 ):
                                            /* Set End-point 3 IN NAK */
                                            USBOTG_FS->UEP3_TX_CTRL = USBFS_UEP_T_RES_NAK;
                                            break;

                                        default:
                                            errflag = 0xFF;
                                            break;
                                    }
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                            }
                            else
                            {
                                errflag = 0xFF;
                            }
                            break;

                            /* set or enable one usb feature */
                        case USB_SET_FEATURE:
                            if( ( USBFS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
                            {
                                /* Set Device Feature */
                                if( (uint8_t)( USBFS_SetupReqValue & 0xFF ) == USB_REQ_FEAT_REMOTE_WAKEUP )
                                {
                                    if( MyCfgDescr[ 7 ] & 0x20 )
                                    {
                                        /* Set Wake-up flag, device prepare to sleep */
                                        USBFS_DevSleepStatus |= 0x01;
                                    }
                                    else
                                    {
                                        errflag = 0xFF;
                                    }
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                            }
                            else if( ( USBFS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
                            {
                                /* Set End-point Feature */
                                if( (uint8_t)( USBFS_SetupReqValue & 0xFF ) == USB_REQ_FEAT_ENDP_HALT )
                                {
                                    /* Set end-points status stall */
                                    switch( (uint8_t)( USBFS_SetupReqIndex & 0xFF ) )
                                    {
                                        case ( DEF_UEP_IN | DEF_UEP1 ):
                                            /* Set End-point 1 IN STALL */
                                            USBOTG_FS->UEP1_TX_CTRL = ( USBOTG_FS->UEP1_TX_CTRL & ~USBFS_UEP_T_RES_MASK ) | USBFS_UEP_T_RES_STALL;
                                            break;

                                        case ( DEF_UEP_OUT | DEF_UEP2 ):
                                            /* Set End-point 2 OUT STALL */
                                            USBOTG_FS->UEP2_RX_CTRL = ( USBOTG_FS->UEP2_RX_CTRL & ~USBFS_UEP_R_RES_MASK ) | USBFS_UEP_R_RES_STALL;
                                            break;

                                        case ( DEF_UEP_IN | DEF_UEP3 ):
                                            /* Set End-point 3 IN STALL */
                                            USBOTG_FS->UEP3_TX_CTRL = ( USBOTG_FS->UEP3_TX_CTRL & ~USBFS_UEP_T_RES_MASK ) | USBFS_UEP_T_RES_STALL;
                                            break;

                                        default:
                                            errflag = 0xFF;
                                            break;
                                    }
                                }
                                else
                                {
                                    errflag = 0xFF;
                                }
                            }
                            else
                            {
                                errflag = 0xFF;
                            }
                            break;

                            /* This request allows the host to select another setting for the specified interface  */
                        case USB_GET_INTERFACE:
                            USBFS_EP0_Buf[0] = 0x00;
                            if ( USBFS_SetupReqLen > 1 )
                            {
                                USBFS_SetupReqLen = 1;
                            }
                            break;

                        case USB_SET_INTERFACE:
                            break;

                            /* host get status of specified device/interface/end-points */
                        case USB_GET_STATUS:
                            USBFS_EP0_Buf[ 0 ] = 0x00;
                            USBFS_EP0_Buf[ 1 ] = 0x00;
                            if ( ( USBFS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_DEVICE )
                            {
                                if( USBFS_DevSleepStatus & 0x01 )
                                {
                                    USBFS_EP0_Buf[ 0 ] = 0x02;
                                }
                            }
                            else if( ( USBFS_SetupReqType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
                            {
                                switch( (uint8_t)( USBFS_SetupReqIndex & 0xFF ) )
                                {
                                    case ( DEF_UEP_IN | DEF_UEP1 ):
                                        if( ( (USBOTG_FS->UEP1_TX_CTRL) & USBFS_UEP_T_RES_MASK ) == USBFS_UEP_T_RES_STALL )
                                        {
                                            USBFS_EP0_Buf[ 0 ] = 0x01;
                                        }
                                        break;

                                    case ( DEF_UEP_OUT | DEF_UEP2 ):
                                        if( ( (USBOTG_FS->UEP2_RX_CTRL) & USBFS_UEP_R_RES_MASK ) == USBFS_UEP_R_RES_STALL )
                                        {
                                            USBFS_EP0_Buf[ 0 ] = 0x01;
                                        }
                                        break;

                                    case ( DEF_UEP_IN | DEF_UEP3 ):
                                        if( ( (USBOTG_FS->UEP3_TX_CTRL) & USBFS_UEP_T_RES_MASK ) == USBFS_UEP_T_RES_STALL )
                                        {
                                            USBFS_EP0_Buf[ 0 ] = 0x01;
                                        }
                                        break;

                                    default:
                                        errflag = 0xFF;
                                        break;
                                }
                            }
                            else
                            {
                                errflag = 0xFF;
                            }

                            if( USBFS_SetupReqLen > 2 )
                            {
                                USBFS_SetupReqLen = 2;
                            }

                            break;

                        default:
                            errflag = 0xFF;
                            break;
                    }
                }
                /* errflag = 0xFF means a request not support or some errors occurred, else correct */
                if( errflag == 0xff)
                {
                    /* if one request not support, return stall */
                    USBOTG_FS->UEP0_TX_CTRL = USBFS_UEP_T_TOG|USBFS_UEP_T_RES_STALL;
                    USBOTG_FS->UEP0_RX_CTRL = USBFS_UEP_R_TOG|USBFS_UEP_R_RES_STALL;
                }
                else
                {
                    /* end-point 0 data Tx/Rx */
                    if( USBFS_SetupReqType & DEF_UEP_IN )
                    {
                        /* tx */
                        len = (USBFS_SetupReqLen>DEF_USBD_UEP0_SIZE) ? DEF_USBD_UEP0_SIZE : USBFS_SetupReqLen;
                        USBFS_SetupReqLen -= len;
                        USBOTG_FS->UEP0_TX_LEN  = len;
                        USBOTG_FS->UEP0_TX_CTRL = USBFS_UEP_T_TOG|USBFS_UEP_T_RES_ACK;
                    }
                    else
                    {
                        /* rx */
                        if( USBFS_SetupReqLen == 0 )
                        {
                            USBOTG_FS->UEP0_TX_LEN  = 0;
                            USBOTG_FS->UEP0_TX_CTRL = USBFS_UEP_T_TOG|USBFS_UEP_T_RES_ACK;
                        }
                        else
                        {
                            USBOTG_FS->UEP0_RX_CTRL = USBFS_UEP_R_TOG|USBFS_UEP_R_RES_ACK;
                        }
                    }
                }
                break;

                /* Sof pack processing */
            case USBFS_UIS_TOKEN_SOF:
                break;

            default :
                break;
        }
        USBOTG_FS->INT_FG = USBFS_UIF_TRANSFER;
    }
    else if( intflag & USBFS_UIF_BUS_RST )
    {
        /* usb reset interrupt processing */
        USBFS_DevConfig = 0;
        USBFS_DevAddr = 0;
        USBFS_DevSleepStatus = 0;
        USBFS_DevEnumStatus = 0;

        USBOTG_FS->DEV_ADDR = 0;
        USBFS_Device_Endp_Init( );
        USBOTG_FS->INT_FG = USBFS_UIF_BUS_RST;
    }
    else if( intflag & USBFS_UIF_SUSPEND )
    {
        /* usb suspend interrupt processing */
        if ( USBOTG_FS->MIS_ST & USBFS_UMS_SUSPEND )
        {
            USBFS_DevSleepStatus |= 0x02;
            if( USBFS_DevSleepStatus == 0x03 )
            {
                /* Handling usb sleep here */
            }
        }
        else
        {
            USBFS_DevSleepStatus &= ~0x02;
        }
        USBOTG_FS->INT_FG = USBFS_UIF_SUSPEND;
    }
    else
    {
        /* other interrupts */
        USBOTG_FS->INT_FG = intflag;
    }
}