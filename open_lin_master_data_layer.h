/*
 * open_lin_master_data_layer.h
 *
 *  Created on: 30.01.2018
 *      Author: ay7vi2
 */

#ifndef OPEN_LIN_OPEN_LIN_MASTER_DATA_LAYER_H_
#define OPEN_LIN_OPEN_LIN_MASTER_DATA_LAYER_H_

#include "open_lin_types.h"
#include "open_lin_network_layer.h"
typedef struct {
	uint32_t frame_slot_us;
	open_lin_frame_slot_t slot;
} t_master_frame_table_item;

typedef enum {
	OPEN_LIN_MASTER_IDLE,
	OPEN_LIN_MASTER_DATA_RX,
	OPEN_LIN_MASTER_TX_DATA,
} t_open_lin_master_state;

void open_lin_master_dl_init(t_master_frame_table_item *p_master_frame_table, l_u8 p_master_frame_table_size);
l_bool open_lin_master_dl_rx(l_u8 rx_byte);
void open_lin_master_dl_handler();
void open_lin_master_dl_rx_callback(open_lin_frame_slot_t* slot);
void open_lin_master_dl_set_state_callback(void (*callback)(t_open_lin_master_state new_state));
l_u32 open_lin_master_dl_get_frame_start_time_us();

/**
 * @brief Pause/resume the master's own schedule (Section 2.6) - while the
 *        network is asleep, the whole bus must stay silent, so open_lin_master_dl_handler()
 *        no-ops until this is cleared again.
 */
void open_lin_master_dl_set_asleep(l_bool asleep);

/**
 * @brief Send a bare wake-up signal (Section 2.6.3): break only, with no
 *        sync/PID/data following - that absence is what distinguishes it from
 *        a real header. Does not resume the schedule by itself; the caller
 *        must wait the spec's minimum settle time before calling
 *        open_lin_master_dl_set_asleep(l_false), so a slower slave has time
 *        to be ready again.
 */
l_bool open_lin_master_dl_send_wake_up(void);

/* this functions by default are handled by open_lin_mastr_dl_handler */
l_bool open_lin_master_data_tx_header(open_lin_frame_slot_t  *slot);
l_bool open_lin_master_data_tx_data(open_lin_frame_slot_t *slot);


#endif /* OPEN_LIN_OPEN_LIN_MASTER_DATA_LAYER_H_ */
