/**
 * @file open_lin_hw.h
 * @author LJ
 * @date 25.01.2018
 * @brief LIN hardware function calls
 *
 */

#ifndef OPEN_LIN_OPEN_LIN_HW_H_
#define OPEN_LIN_OPEN_LIN_HW_H_

#include "open_lin_types.h"
/**
 * @brief Checks if break signal occurred on UART
 *
 * @return @c true is break is detected
 */
l_bool open_lin_hw_check_for_break(void);

/**
 * @brief resets hardware to default state
 *
 * @return @c true is break is detected
 */
void open_lin_hw_reset(void);

void open_lin_set_rx_enabled(l_bool status);

void open_lin_hw_init(void);

l_u32 open_lin_hw_get_time_us();

l_u8 open_lin_hw_tx_data(l_u8* data, l_u8 len);
l_u8 open_lin_hw_rx_data(l_u8 *data, l_u8 len, l_u32 timeout_us);

l_bool open_lin_hw_tx_break(void);

l_bool open_lin_hw_tx_byte(l_u8 data);
l_bool open_lin_hw_rx_byte(l_u8 *data, l_u32 timeout_us);

/**
 * @brief Starts receiving len bytes into buffer and returns at once; the hardware stores the
 *        bytes in the background as they arrive. Cancels a reception still pending.
 *        Poll open_lin_hw_rx_count() for the progress.
 */
void open_lin_hw_rx_start(l_u8 *buffer, l_u8 len);

/**
 * @brief Number of bytes stored since open_lin_hw_rx_start(), also while the reception
 *        is not complete yet (e.g. after a timeout).
 */
l_u8 open_lin_hw_rx_count(void);

/** @brief Cancels the reception started with open_lin_hw_rx_start(), without disturbing the bus. */
void open_lin_hw_rx_abort(void);

/**
 * @brief Starts sending len bytes and returns at once; the hardware sends them in the background.
 *        The buffer must stay unchanged until open_lin_hw_tx_is_done() reports true.
 * @return l_false when the transmission could not be started
 */
l_bool open_lin_hw_tx_start(const l_u8 *buffer, l_u8 len);

/** @brief l_true when the transmission started with open_lin_hw_tx_start() has been handed to the line. */
l_bool open_lin_hw_tx_is_done(void);

#ifdef OPEN_LIN_AUTO_BAUD
/**
 * @brief Starts auto baud procedure
 *
 */
void open_lin_hw_set_auto_baud(void);

#endif /* OPEN_LIN_AUTO_BAUD */

#endif /* OPEN_LIN_OPEN_LIN_HW_H_ */
