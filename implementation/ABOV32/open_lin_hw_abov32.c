/*
 * open_lin_hw_abov32.c
 *
 */
#include "abov_config.h"
#include "hal_pcu.h"

#include "open_lin_types.h"
#include "open_lin_hw.h"

#define PRINT_RAW_DATA

#define COLOR_RESET   "\033[0m"
#define COLOR_BOLD    "\033[1m"
#define COLOR_RED     "\033[31m"COLOR_BOLD
#define COLOR_GREEN   "\033[32m"COLOR_BOLD
#define COLOR_YELLOW  "\033[33m"COLOR_BOLD
#define COLOR_BLUE    "\033[34m"COLOR_BOLD
#define COLOR_PURPLE  "\033[35m"COLOR_BOLD
#define COLOR_CYAN    "\033[36m"COLOR_BOLD

#if USART_LIN
#include "hal_usart.h"
#else
#include "hal_uart.h"
#endif

extern uint32_t GetSysTick(void);

bool s_bBreakDetect;
static bool s_bRxDone;
static bool s_bTxDone;

static void PRV_IRQHandler(uint32_t un32Event, void *pContext)
{
#if USART_LIN
    if(un32Event & USART_EVENT_RX_DONE)
    {
        s_bRxDone = true;
    }

    if(un32Event & USART_EVENT_TX_DONE)
    {
        s_bTxDone = true;
    }
#else
    if(un32Event & UART_EVENT_BREAK_ERROR)
    {
        printf(COLOR_GREEN"LIN signal detected.\n"COLOR_RESET);
        SET_BIT(UART1,false,UART,IER,RLSIE);
        s_bBreakDetect = true;
    }

    if(un32Event & UART_EVENT_RX_DONE)
    {
        s_bRxDone = true;
    }

    if(un32Event & UART_EVENT_TX_DONE)
    {
        s_bTxDone = true;
    }
#endif
}

static void Delay(l_u32 delay_us)
{
    uint32_t start_time_us = open_lin_hw_get_time_us();
    while(open_lin_hw_get_time_us() - start_time_us <= delay_us) ;
}

l_u32 open_lin_hw_get_time_us()
{
    return GetSysTick();
}

void open_lin_frame_set_auto_baud(void)
{
    ;
}

void open_lin_hw_reset(void)
{
#if USART_LIN
    USART_CFG_t tCfg =
    {
        .eMode = USART_MODE_LIN,
        .un32BaudRate = 19200,
        .tCfg.tUart.eData = USART_DATA_8,
        .tCfg.tUart.eParity = USART_PARITY_NONE,
        .tCfg.tUart.eStop = USART_STOP_1,
        .tCfg.tUart.bDoubleSpeed = false,
    };

    HAL_PCU_SetAltMode(PCU_ID_C, PCU_PIN_ID_7, PCU_ALT_4);
    HAL_PCU_SetAltMode(PCU_ID_C, PCU_PIN_ID_8, PCU_ALT_4);

    HAL_USART_Init(USART_ID_1);
    HAL_USART_SetConfig(USART_ID_1, &tCfg);
    HAL_USART_SetIRQ(USART_ID_1, USART_OPS_INTR, PRV_IRQHandler, NULL, 3);
#else
    UART_CLK_CFG_t cClkCfg =
    {
        .eClk = UART_CLK_MCCR,
        .eMccr = UART_CLK_MCCR_MCLK,
        .un8MccrDiv = 2
    };

    UART_CFG_t tCfg =
    {
        .un32BaudRate = 19200,
        .eData = UART_DATA_8,
        .eParity = UART_PARITY_NONE,
        .eStop = UART_STOP_1,
        .bIntrLSEnable = false
    };

    HAL_PCU_SetAltMode(PCU_ID_B, PCU_PIN_ID_6, PCU_ALT_1);
    HAL_PCU_SetAltMode(PCU_ID_B, PCU_PIN_ID_7, PCU_ALT_1);

    HAL_UART_Init(UART_ID_1);
    HAL_UART_SetClkConfig(UART_ID_1, &cClkCfg);
    HAL_UART_SetConfig(UART_ID_1, &tCfg);
    HAL_UART_SetIRQ(UART_ID_1, UART_OPS_INTR, PRV_IRQHandler, NULL, 3);
    HAL_UART_SetIFDelay(UART_ID_1, 7);
#endif
}

void open_lin_set_rx_enabled(l_bool v)
{
#if USART_LIN
    SET_BIT(USART11,v,USART1n,CR1,RXEn);
#else
    SET_BIT(UART1,v,UART,IER,DRIE);
#endif
}

l_bool open_lin_hw_check_for_break(void)
{
#if USART_LIN
    if(s_bBreakDetect == false)
    {
        s_bBreakDetect = GET_BIT(USART11,USART1n,CR3,RXBRKnIFLAG);
        if(s_bBreakDetect)
        {
            SET_BIT(USART11,true,USART1n,CR3,RXBRKnIFLAG);
            printf(COLOR_GREEN"LIN signal detected.\n"COLOR_RESET);
        }
    }
#else
    if(s_bBreakDetect == false)
    {
        SET_BIT(UART1,true,UART,IER,RLSIE);
    }
#endif
    return s_bBreakDetect;
}

l_bool open_lin_hw_tx_break(void)
{
#if USART_LIN
    bool result = false;

    SET_BIT(USART11,true,USART1n,CR3,TXBRKn);
    while(!GET_BIT(USART11,USART1n,CR3,TXBRKnIFLAG))
    {
        SET_BIT(USART11,true,USART1n,CR3,TXBRKnIFLAG);
        result = true;
        printf(COLOR_GREEN"LIN signal generated.\n"COLOR_RESET);
        Delay(1000);
        break;
    }
    return result;
#else
    // ALT : UART -> GPIO
    HAL_PCU_SetAltMode(PCU_ID_B, PCU_PIN_ID_6, PCU_ALT_0);
    HAL_PCU_SetInOutMode(PCU_ID_B, PCU_PIN_ID_6, PCU_INOUT_OUTPUT_PUSH_PULL);

    // Generated LIN break + delimiter
    HAL_PCU_SetOutputBit(PCU_ID_B, PCU_PIN_ID_6, PCU_OUTPUT_BIT_CLEAR);
    Delay(700);
    HAL_PCU_SetOutputBit(PCU_ID_B, PCU_PIN_ID_6, PCU_OUTPUT_BIT_SET);
    Delay(57);

    // ALT  : GPIO -> UART
    HAL_PCU_SetAltMode(PCU_ID_B, PCU_PIN_ID_6, PCU_ALT_1);
    printf(COLOR_GREEN"LIN signal generated.\n"COLOR_RESET);

    return true;
#endif
}

l_bool open_lin_hw_tx_byte(l_u8 byte)
{
    HAL_ERR_e eErr;
    s_bTxDone = false;

#if USART_LIN
    eErr = HAL_USART_Transmit(USART_ID_1, &byte, 1, false);
#else
    eErr = HAL_UART_Transmit(UART_ID_1, &byte, 1, false);
#endif

    if(eErr == HAL_ERR_OK)
    {
        while(!s_bTxDone) ;
#if defined(PRINT_RAW_DATA)
        printf(COLOR_PURPLE"[byte] %02X\n"COLOR_RESET, byte);
#endif
    }

    return s_bTxDone;
}

l_u8 open_lin_hw_tx_data(l_u8* data, l_u8 len)
{
    HAL_ERR_e eErr;
    s_bTxDone = false;

#if USART_LIN
    eErr = HAL_USART_Transmit(USART_ID_1, data, len, false);
#else
    eErr = HAL_UART_Transmit(UART_ID_1, data, len, false);
#endif

    if(eErr == HAL_ERR_OK)
    {
        while(!s_bTxDone) ;
#if defined(PRINT_RAW_DATA)
        printf(COLOR_PURPLE"[data] ");
        for(int i = 0 ; i < len ; i++)
            printf("%02X ", data[i]);
        printf("\n"COLOR_RESET);
#endif
    }
    return s_bTxDone;
}

l_bool open_lin_hw_rx_byte(l_u8 *data, l_u32 timeout_us)
{
    uint32_t start_time_us;

    s_bRxDone = false;
    start_time_us = open_lin_hw_get_time_us();

#if USART_LIN
    HAL_USART_Receive(USART_ID_1, data, 1, false);
#else
    HAL_UART_Receive(UART_ID_1, data, 1, false);
#endif

    while(!s_bRxDone && (open_lin_hw_get_time_us() - start_time_us <= timeout_us)) ;
#if defined(PRINT_RAW_DATA)
    if(s_bRxDone)
    {
        printf(COLOR_YELLOW"[byte] %02X\n"COLOR_RESET, *data);
    }
#endif
    return s_bRxDone;
}

l_u8 open_lin_hw_rx_data(l_u8 *data, l_u8 len, l_u32 timeout_us)
{
    uint32_t start_time_us;

    s_bRxDone = false;
    start_time_us = open_lin_hw_get_time_us();

#if USART_LIN
    HAL_USART_Receive(USART_ID_1, data, len, false);
#else
    HAL_UART_Receive(UART_ID_1, data, len, false);
#endif

    while(!s_bRxDone && (open_lin_hw_get_time_us() - start_time_us <= timeout_us)) ;
#if defined(PRINT_RAW_DATA)
    if(s_bRxDone)
    {
        printf(COLOR_YELLOW"[data] ");
        for(int i = 0 ; i < len ; i++)
            printf("%02X ", data[i]);
        printf("\n"COLOR_RESET);
    }
#endif
    return s_bRxDone ? len : 0;
}
