/*
 * open_lin_hw_abov32.c
 *
 */
#include "abov_config.h"
#include "hal_pcu.h"

#include "open_lin_types.h"
#include "open_lin_hw.h"

// #define PRINT_RAW_DATA
// #define PRINT_BREAK_SIGNAL

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

#include "hal_timer1.h"

/* TIMER1 channel dedicated to the LIN microsecond time base.
 * Timer1 is prescaled down to LIN_TIME_TICK_HZ (100kHz -> 1 tick = 10us),
 * computed from the actual configured SystemCoreClock (HSI-derived, see
 * ProductConfig/<chip>/Config/startup_<chip>.h's HSI_CLOCK) rather than a
 * hardcoded divisor, so it stays correct if the MCLK/HCLK setup ever changes.
 * The 16bit HW counter wraps every 65536 ticks (655.36ms); s_un32TimeHigh
 * extends it to a genuine 32bit tick count via the periodic Match-A event
 * (used in place of an overflow interrupt so the same code works on parts
 * without a dedicated OVF flag, e.g. A31S13x). This extension is required
 * for correctness, not just extra range: any two open_lin_hw_get_time_us()
 * reads that straddle a 655ms wrap boundary would otherwise be corrupted by
 * plain unsigned subtraction, regardless of how short the measured span is. */
#define LIN_TIME_TIMER1_ID    TIMER1_ID_0
#define LIN_TIME_US_PER_TICK  10u
#define LIN_TIME_TICK_HZ      (1000000u / LIN_TIME_US_PER_TICK)

static volatile uint32_t s_un32TimeHigh;

/* set from the UART IRQ and polled by the main loop: must be volatile, or -O2/-O3 may hoist the read out of the wait loops */
volatile bool s_bBreakDetect;
static volatile bool s_bRxDone;
static volatile bool s_bTxDone;

static void PRV_LinTimer_IRQHandler(uint32_t un32Event, void *pContext)
{
    if (un32Event & TIMER1_EVENT_PERIODIC_MATCH)
    {
        s_un32TimeHigh++;
    }
}

static void PRV_LinUart_IRQHandler(uint32_t un32Event, void *pContext)
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
        SET_BIT(UART1,false,UART,IER,RLSIE);
        s_bBreakDetect = true;
#if defined(PRINT_BREAK_SIGNAL)
        printf(COLOR_GREEN"LIN signal detected.\n"COLOR_RESET);
#endif
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

static void PRV_LinTimerInit(void)
{
    /* NOTE: Timer1's version-specific private headers (Vx/V1x/Fx/F1x, where a
     * TIMER1_FEATURE_USE_MCCR_CLOCK-style capability flag would live) are only
     * included by hal_timer1.c itself via hal_timer1_prv.h - they are NOT
     * visible here, since this file only sees the public hal_timer1.h. So
     * whether this Timer1 instance has a real MCCR clock mux is declared
     * explicitly per build target via TIMER1_PERI_CLOCK_ONLY (see each
     * target's CMakeLists.txt target_compile_definitions), rather than
     * inferred from a chip/product macro. */
#if (TIMER1_PERI_CLOCK_ONLY == 1)
    TIMER1_CLK_CFG_t tClk =
    {
        .eClk             = TIMER1_CLK_PCLK,
        .uSubClk.ePClkDiv = TIMER1_PCLK_DIV_2,  /* ignored by this HAL variant */
        .un16PreScale     = (uint16_t)(SystemCoreClock / LIN_TIME_TICK_HZ),
    };
#else
    TIMER1_CLK_CFG_t tClk =
    {
        .eClk             = TIMER1_CLK_MCCR,
        .uSubClk.eMccr    = TIMER1_CLK_MCCR_MCLK,
        .un8MccrDiv       = 10,    /* MCLK/10 -> fed into un16PreScale below */
        .un16PreScale     = (uint16_t)(SystemCoreClock / 10u / LIN_TIME_TICK_HZ),
    };
#endif
    TIMER1_CFG_t tCfg =
    {
        .eMode            = TIMER1_MODE_PERIODIC,
        .bIntrEnable      = true,   /* Match-A fires exactly at the 16bit wrap (DataA=0xFFFF);
                                       used as our portable stand-in for an overflow interrupt */
        .bOVFIntrEnable   = false,
        .utData.tGRD.un16DataA = 0xFFFFu,  /* let the 16bit counter run its full range */
        .utData.tGRD.un16DataB = 0xFFFFu,
    };

    HAL_TIMER1_Init(LIN_TIME_TIMER1_ID);
    HAL_TIMER1_SetClkConfig(LIN_TIME_TIMER1_ID, &tClk);
    HAL_TIMER1_SetConfig(LIN_TIME_TIMER1_ID, &tCfg);
    HAL_TIMER1_SetIRQ(LIN_TIME_TIMER1_ID, TIMER1_OPS_INTR, PRV_LinTimer_IRQHandler, NULL, 3);
    HAL_TIMER1_Start(LIN_TIME_TIMER1_ID);
}

l_u32 open_lin_hw_get_time_us(void)
{
    uint32_t un32High1, un32High2, un32Low;

    do
    {
        un32High1 = s_un32TimeHigh;
        HAL_TIMER1_GetData(LIN_TIME_TIMER1_ID, TIMER1_DATA_CNT, &un32Low);
        un32High2 = s_un32TimeHigh;
    } while (un32High1 != un32High2);  /* retry if a wrap happened mid-read */

    return (l_u32)(((un32High2 << 16) | (un32Low & 0xFFFFu)) * LIN_TIME_US_PER_TICK);
}

void open_lin_frame_set_auto_baud(void)
{
    ;
}

void open_lin_hw_reset(void)
{
    PRV_LinTimerInit();

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
    HAL_USART_SetIRQ(USART_ID_1, USART_OPS_INTR, PRV_LinUart_IRQHandler, NULL, 3);
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
    HAL_UART_SetIRQ(UART_ID_1, UART_OPS_INTR, PRV_LinUart_IRQHandler, NULL, 3);
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
#if defined(PRINT_BREAK_SIGNAL)
            printf(COLOR_GREEN"LIN signal detected.\n"COLOR_RESET);
#endif
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
        Delay(1000);
#if defined(PRINT_BREAK_SIGNAL)
        printf(COLOR_GREEN"LIN signal generated.\n"COLOR_RESET);
#endif
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

    // ALT  : GPIO -> UART
    HAL_PCU_SetAltMode(PCU_ID_B, PCU_PIN_ID_6, PCU_ALT_1);
    Delay(60);
#if defined(PRINT_BREAK_SIGNAL)
    printf(COLOR_GREEN"LIN signal generated.\n"COLOR_RESET);
#endif
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

/* Non-blocking reception of one frame: the HAL's interrupt handler stores the bytes into uffer as they
 * arrive, nothing is called from this layer in interrupt context. */
static l_u8 s_rx_len;
#if defined(PRINT_RAW_DATA)
/* The bytes of the reception / transmission in progress are printed once, when its done flag has gone true
 * (the callback of the HAL sets it): printing from the polling functions below, never from the interrupt. */
static const l_u8 *s_rx_log_buffer;
static l_bool s_rx_logged = l_true;
static const l_u8 *s_tx_log_buffer;
static l_u8 s_tx_log_len;
static l_bool s_tx_logged = l_true;
#endif

void open_lin_hw_rx_data(l_u8 *buffer, l_u8 len)
{
    s_rx_len = len;
    s_bRxDone = false;
#if defined(PRINT_RAW_DATA)
    s_rx_log_buffer = buffer;
    s_rx_logged = l_false;
#endif
#if USART_LIN
    (void)HAL_USART_AbortRx(USART_ID_1);
    (void)HAL_USART_Receive(USART_ID_1, buffer, len, false);
#else
    (void)HAL_UART_AbortRx(UART_ID_1);
    (void)HAL_UART_Receive(UART_ID_1, buffer, len, false);
#endif
}

/* Non-blocking transmission: the HAL's interrupt handler chains the bytes, s_bTxDone is set by its callback. */
l_bool open_lin_hw_tx_data(const l_u8 *buffer, l_u8 len)
{
    HAL_ERR_e eErr;

    s_bTxDone = false;
#if defined(PRINT_RAW_DATA)
    s_tx_log_buffer = buffer;
    s_tx_log_len = len;
    s_tx_logged = l_false;
#endif
#if USART_LIN
    eErr = HAL_USART_Transmit(USART_ID_1, (uint8_t *)buffer, len, false);
#else
    eErr = HAL_UART_Transmit(UART_ID_1, (uint8_t *)buffer, len, false);
#endif
    return (eErr == HAL_ERR_OK) ? l_true : l_false;
}

l_bool open_lin_hw_tx_is_done(void)
{
#if defined(PRINT_RAW_DATA)
    if (s_bTxDone && (s_tx_logged == l_false))
    {
        s_tx_logged = l_true;
        printf(COLOR_PURPLE"[tx] ");
        for (l_u8 i = 0u; i < s_tx_log_len; i++)
        {
            printf("%02X ", s_tx_log_buffer[i]);
        }
        printf("\n"COLOR_RESET);
    }
#endif
    return s_bTxDone ? l_true : l_false;
}

l_u8 open_lin_hw_rx_count(void)
{
#if defined(PRINT_RAW_DATA)
    if (s_bRxDone && (s_rx_logged == l_false))
    {
        s_rx_logged = l_true;
        printf(COLOR_YELLOW"[rx] ");
        for (l_u8 i = 0u; i < s_rx_len; i++)
        {
            printf("%02X ", s_rx_log_buffer[i]);
        }
        printf("\n"COLOR_RESET);
    }
#endif
#if USART_LIN
    uint32_t count = 0u;

    (void)HAL_USART_GetRxCount(USART_ID_1, &count);
    return (count > s_rx_len) ? s_rx_len : (l_u8)count;
#else
    uint32_t count = 0u;

    (void)HAL_UART_GetRxCount(UART_ID_1, &count);
    return (count > s_rx_len) ? s_rx_len : (l_u8)count;
#endif
}

void open_lin_hw_rx_abort(void)
{
#if USART_LIN
    (void)HAL_USART_AbortRx(USART_ID_1);
#else
    (void)HAL_UART_AbortRx(UART_ID_1);
#endif
}
