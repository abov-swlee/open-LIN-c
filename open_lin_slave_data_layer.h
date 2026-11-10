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

/**
 * @brief Section 2.7.3: designate which of this slave's own transmitted
 *        frames carries the mandatory one-bit response_error status signal,
 *        and where in that frame's data it lives. The library ORs the
 *        current status into that bit right before the frame goes out
 *        (after the app's open_lin_net_tx_handler() fills the rest of the
 *        data, before the checksum is computed), and clears it once that
 *        specific frame has been transmitted successfully - the app doesn't
 *        need to touch it at all beyond calling this once at init.
 * @param[in] pid         The (unparitied) id of one of this node's own TX frames.
 * @param[in] byte_index  Index into that frame's data (must be < its data_length).
 * @param[in] bit_index   Bit position within that byte (0-7).
 */
void open_lin_slave_dl_set_response_error_frame(open_lin_pid_t pid, l_u8 byte_index, l_u8 bit_index);

/**
 * @brief Section 2.7.1: current value of this node's response_error status -
 *        exposed so the application can also act on it directly (e.g. enter
 *        a limp-home mode, which 2.7.1 names as the point of all this), not
 *        just see it folded into the designated frame on the bus.
 */
l_bool open_lin_slave_dl_get_response_error(void);

/**
 * @brief Section 2.3.3.2: declare that this node publishes one of the unconditional
 *        frames associated with an event-triggered frame. When a header with
 *        event_pid arrives and the application has flagged an update with
 *        open_lin_slave_dl_event_updated(), the library answers with the
 *        associated frame's data - its data byte 0 replaced by that frame's
 *        protected id, the checksum computed over event_pid - and then
 *        considers the signal no longer updated. With no update the slot is
 *        left silent. Event-triggered responses never set response_error
 *        (Section 2.7.2).
 * @param[in] event_pid       The (unparitied) id of the event-triggered frame.
 * @param[in] associated_pid  The id of this node's own TX frame (must be in the
 *                            slot array); its data byte 0 must be reserved.
 */
void open_lin_slave_dl_set_event_frame(open_lin_pid_t event_pid, open_lin_pid_t associated_pid);

/**
 * @brief A signal carried in the associated frame was updated, so the next
 *        event-triggered header gets a response. Also cleared when the
 *        associated frame is sent as an ordinary unconditional frame.
 */
void open_lin_slave_dl_event_updated(void);

/**
 * @brief Debug/test only: corrupt the checksum of the next event-triggered
 *        response, which is what the master sees when two slaves answer at
 *        once - lets collision handling be tried with a single slave board.
 *        The update stays pending, as it would for a real collision.
 */
void open_lin_slave_dl_debug_event_garble(void);

/**
 * @brief Debug/test only: force response_error true without a real protocol
 *        error, so its appearance in the designated frame (and automatic
 *        clearing once that frame is next sent) can be observed on real
 *        hardware without needing to physically fault-inject a checksum or
 *        HW TX error.
 */
void open_lin_slave_dl_debug_force_response_error(void);

#endif /* OPEN_LIN_SLAVE_DATA_LAYER_H_ */
