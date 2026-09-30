/**
 * @file open_lin_types.h
 * @brief LIN types declaration according to LIN Specification Package
 */
#ifndef OPEN_LIN_SLAVE_DATA_LAYER_H_
#define OPEN_LIN_SLAVE_DATA_LAYER_H_

#include "open_lin_types.h"

typedef enum {
	OPEN_LIN_SLAVE_IDLE,
	OPEN_LIN_SLAVE_SYNC_RX,
	OPEN_LIN_SLAVE_PID_RX,
	OPEN_LIN_SLAVE_DATA_RX,
	OPEN_LIN_SLAVE_DATA_TX
} t_open_lin_slave_state;

/**
 * @brief Init LIN slave component
 */
void open_lin_slave_init(void);

/**
 * @brief Reset LIN slave component
 */
void open_lin_slave_reset(void);

/**
 * @brief byte reception handler function
 * @param rx_byte
 */
void open_lin_slave_rx_header(l_u8 rx_byte);

void open_lin_slave_set_state_callback(void (*callback)(t_open_lin_slave_state new_state));

/**
 * @brief Send a bare wake-up signal (Section 2.6.2): break only, no sync/PID/
 *        data following. Any node may originate a wake-up, not just the
 *        master - unlike open_lin_master_dl_send_wake_up(), this doesn't
 *        pause/resume anything on this node: per spec, the slave that
 *        originates the signal is ready to receive/transmit immediately, and
 *        it's the master's job to notice and resume transmitting headers.
 */
l_bool open_lin_slave_dl_send_wake_up(void);

#endif /* OPEN_LIN_SLAVE_DATA_LAYER_H_ */
