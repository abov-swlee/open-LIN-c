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
#include "open_lin_event_trigger.h"
typedef struct open_lin_master_frame_table_item_s {
	uint32_t frame_slot_us;
	open_lin_frame_slot_t slot;
	/**
	 * @brief NULL for a plain unconditional frame entry (existing behavior,
	 *        unchanged). Non-NULL marks this entry as event-triggered: `slot`
	 *        still carries the event-triggered frame's own PID (for the
	 *        header) and `event_trigger` lists its candidate responders.
	 */
	const open_lin_event_trigger_t *event_trigger;
} t_master_frame_table_item;

typedef enum {
	OPEN_LIN_MASTER_IDLE,
	OPEN_LIN_MASTER_DATA_RX,
	OPEN_LIN_MASTER_TX_DATA,
} t_open_lin_master_state;

void open_lin_master_dl_init(t_master_frame_table_item *p_master_frame_table, l_u8 p_master_frame_table_size);

/**
 * @brief Request a switch to another schedule table (LDF Schedule_tables).
 *        The switch happens between frames: a frame slot already in progress
 *        finishes first, then the new table starts at its first entry.
 *        A second request before the switch happens replaces the first.
 * @param[in] p_table  The new table, must stay valid while it is in use.
 * @param[in] p_size   Number of entries in p_table.
 * @return l_true if accepted, l_false (request ignored) for a NULL or empty table.
 */
l_bool open_lin_master_dl_switch_schedule_table(t_master_frame_table_item *p_table, l_u8 p_size);
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
 * @brief Query whether the master's schedule is currently paused (Section
 *        2.6) - lets the app decide whether to poll the schedule handler or
 *        listen for an incoming wake-up signal instead, see main()'s loop.
 */
l_bool open_lin_master_dl_is_asleep(void);

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
