/**
 * @file open_lin_slave_data_layer.c
 * @brief LIN slave data layer implementation
 */

#include "open_lin_network_layer.h"
#include "open_lin_data_layer.h"
#include "open_lin_types.h"
#include "open_lin_hw.h"
#include "open_lin_slave_data_layer.h"

/**
 * @brief Internal states definition
 */

static t_open_lin_slave_state open_lin_slave_state;
static void (*open_lin_slave_state_callback)(t_open_lin_slave_state new_state);

static l_u8 open_lin_slave_state_data_count;

/* Section 2.7.3: response_error status management. Set whenever a frame this
 * node owns (one of its own RX or TX frames, matched via open_lin_slave_set_lin_frame())
 * has an error in its response; cleared once the designated frame carrying the
 * bit is itself transmitted successfully. Event-triggered frame responses are
 * explicitly excluded (2.7.2): open_lin_slave_event_response() never sets it. */
static l_bool s_response_error = l_false;
static l_bool s_response_error_frame_configured = l_false;
static open_lin_pid_t s_response_error_pid;
static l_u8 s_response_error_byte_index;
static l_u8 s_response_error_bit_index;

void open_lin_slave_dl_set_response_error_frame(open_lin_pid_t pid, l_u8 byte_index, l_u8 bit_index)
{
	s_response_error_pid = pid;
	s_response_error_byte_index = byte_index;
	s_response_error_bit_index = bit_index;
	s_response_error_frame_configured = l_true;
}

void open_lin_slave_dl_debug_force_response_error(void)
{
	s_response_error = l_true;
}

/* Section 2.3.3.2: the event-triggered frames this node takes part in, one entry per associated frame it publishes */
typedef struct
{
	open_lin_pid_t event_pid;
	open_lin_pid_t associated_pid;
	l_bool updated;
} t_slave_event_entry;

static t_slave_event_entry s_events[OPEN_LIN_SLAVE_MAX_EVENT_FRAMES];
static l_u8 s_event_count = 0u;
static l_bool s_event_garble = l_false;

/* A response is sent in the background: data and checksum are copied into one buffer the hardware sends in one
 * go (open_lin_hw_tx_data), and what has to happen once it is out is done by open_lin_slave_poll(). The
 * receiver stays off while it goes out, so the node does not read its own response. */
typedef enum
{
	SLAVE_TX_NONE,
	SLAVE_TX_FRAME, /* an ordinary frame of this node */
	SLAVE_TX_EVENT  /* the response to an event-triggered header */
} t_slave_tx_kind;

static t_slave_tx_kind s_tx_kind = SLAVE_TX_NONE;
static open_lin_pid_t s_tx_pid;
static t_slave_event_entry *s_tx_event = (t_slave_event_entry *)0;
static l_bool s_tx_garbled = l_false;
static l_u32 s_tx_started_us = 0u;
static l_u8 s_tx_stage[OPEN_LIN_MAX_FRAME_LENGTH + 1u];

static l_bool slave_tx_start(const l_u8 *data, l_u8 length, l_u8 checksum)
{
	(void)open_lin_memcpy(s_tx_stage, data, length);
	s_tx_stage[length] = checksum;
	if (open_lin_hw_tx_data(s_tx_stage, (l_u8)(length + 1u)) == l_false)
	{
		return l_false;
	}
	s_tx_started_us = open_lin_hw_get_time_us();
	return l_true;
}

static t_slave_event_entry *event_by_associated_pid(open_lin_pid_t associated_pid)
{
	l_u8 i;
	for (i = 0u; i < s_event_count; i++)
	{
		if (s_events[i].associated_pid == associated_pid)
		{
			return &s_events[i];
		}
	}
	return (t_slave_event_entry *)0;
}

void open_lin_slave_dl_clear_event_frames(void)
{
	s_event_count = 0u;
	s_event_garble = l_false;
}

l_bool open_lin_slave_dl_set_event_frame(open_lin_pid_t event_pid, open_lin_pid_t associated_pid)
{
	t_slave_event_entry *entry = event_by_associated_pid(associated_pid);

	if (entry == (t_slave_event_entry *)0)
	{
		if (s_event_count >= OPEN_LIN_SLAVE_MAX_EVENT_FRAMES)
		{
			return l_false;
		}
		entry = &s_events[s_event_count];
		s_event_count++;
	}
	entry->event_pid = event_pid;
	entry->associated_pid = associated_pid;
	entry->updated = l_false;
	return l_true;
}

void open_lin_slave_dl_event_updated(open_lin_pid_t associated_pid)
{
	t_slave_event_entry *entry = event_by_associated_pid(associated_pid);

	if (entry != (t_slave_event_entry *)0)
	{
		entry->updated = l_true;
	}
}

void open_lin_slave_dl_debug_event_garble(void)
{
	s_event_garble = l_true;
}

/* Is this a header of an event-triggered frame this node takes part in? */
static l_bool is_event_pid(open_lin_pid_t pid)
{
	l_u8 i;
	for (i = 0u; i < s_event_count; i++)
	{
		if (s_events[i].event_pid == pid)
		{
			return l_true;
		}
	}
	return l_false;
}

/* Answer (or stay silent on) a header carrying an event-triggered frame's id.
 * @return l_true when a response is on its way (open_lin_slave_poll() finishes it), l_false when the node stays silent */
static l_bool open_lin_slave_event_response(open_lin_pid_t event_pid)
{
	open_lin_frame_slot_t *slot;
	t_slave_event_entry *entry = (t_slave_event_entry *)0;
	l_u8 checksum;
	l_u8 i;

	for (i = 0u; (i < s_event_count) && (entry == (t_slave_event_entry *)0); i++)
	{
		if ((s_events[i].event_pid == event_pid) && (s_events[i].updated == l_true))
		{
			entry = &s_events[i];
		}
	}
	if (entry == (t_slave_event_entry *)0)
	{
		return l_false; /* none of this node's frames under this event were updated */
	}
	slot = open_lin_net_get_slot(entry->associated_pid);
	if ((slot == OPEN_LIN_NET_SLOT_EMPTY) || (slot->frame_type != OPEN_LIN_FRAME_TYPE_TRANSMIT) || (slot->data_length == 0u))
	{
		return l_false;
	}

	open_lin_set_rx_enabled(l_false); /* don't receive our own response */
	open_lin_net_tx_handler(entry->associated_pid);
	slot->data_ptr[0] = open_lin_data_layer_parity(entry->associated_pid);
	checksum = open_lin_data_layer_checksum(event_pid, slot->checksum_type, slot->data_length, slot->data_ptr);
	if (s_event_garble == l_true)
	{
		checksum = (l_u8)(checksum ^ 0xFFu);
	}

	if (slave_tx_start(slot->data_ptr, slot->data_length, checksum) == l_false)
	{
		open_lin_error_handler(OPEN_LIN_SLAVE_ERROR_HW_TX); /* not counted in response_error, Section 2.7.2 */
		return l_false;
	}
	s_tx_kind = SLAVE_TX_EVENT;
	s_tx_event = entry;
	s_tx_garbled = s_event_garble;
	return l_true;
}
l_bool open_lin_slave_dl_get_response_error(void)
{
	return s_response_error;
}

static void open_lin_slave_set_state(t_open_lin_slave_state new_state)
{
	open_lin_slave_state = new_state;
	if (open_lin_slave_state_callback) {
		open_lin_slave_state_callback(open_lin_slave_state);
	}
}

static l_bool open_lin_slave_set_lin_frame(t_open_lin_data_layer_frame *frame){
	l_bool result = false;
	open_lin_frame_slot_t *frame_slot = open_lin_net_get_slot(frame->pid);
	if (frame_slot != OPEN_LIN_NET_SLOT_EMPTY)
	{
		frame->length = frame_slot->data_length;
		frame->data_ptr = frame_slot->data_ptr;
		frame->checksum_type = frame_slot->checksum_type;
		if (frame_slot->frame_type == OPEN_LIN_FRAME_TYPE_TRANSMIT)
		{
			open_lin_slave_set_state(OPEN_LIN_SLAVE_DATA_TX);
		} else
		{
			open_lin_slave_set_state(OPEN_LIN_SLAVE_DATA_RX);
		}
		result = true;
	} else
	{
		/* do nothing */
	}
	return result;
}

void open_lin_slave_init(void)
{
	open_lin_slave_reset();
}

void open_lin_slave_reset(void){
	s_tx_kind = SLAVE_TX_NONE;
	open_lin_slave_set_state(OPEN_LIN_SLAVE_IDLE);
	open_lin_slave_state_data_count = 0;
	open_lin_set_rx_enabled(l_true);
}

void open_lin_slave_rx_header(l_u8 rx_byte)
{
	static t_open_lin_data_layer_frame open_lin_data_layer_frame;
	static l_u8 open_lin_slave_data_buff[OPEN_LIN_MAX_FRAME_LENGTH];

	if (open_lin_hw_check_for_break() == l_true)
	{
		if (open_lin_slave_state != OPEN_LIN_SLAVE_IDLE)
		{
			s_tx_kind = SLAVE_TX_NONE; /* a response still going out is abandoned: the master started over */
			open_lin_error_handler(OPEN_LIN_SLAVE_ERROR_INVALID_BREAK); /* break should be received only in IDLE state */
		} else {
			/* do nothing, go to SYNC rx anyway*/
		}
        #ifdef OPEN_LIN_HW_BREAK_IS_SYNCH_BYTE
		open_lin_slave_set_state(OPEN_LIN_SLAVE_PID_RX);
        #else
		open_lin_slave_set_state(OPEN_LIN_SLAVE_SYNC_RX);
        #endif
        #ifdef OPEN_LIN_AUTO_BAUD
			open_lin_hw_set_auto_baud();
		#endif
	} else
	{
		switch (open_lin_slave_state) {
			case (OPEN_LIN_SLAVE_SYNC_RX):
			{
				/* synch byte reception do nothing */
				if (rx_byte != OPEN_LIN_SYNCH_BYTE)
				{
					open_lin_error_handler(OPEN_LIN_SLAVE_ERROR_INVALID_SYNCH);
					open_lin_slave_reset();
					break;
				} else
				{
					open_lin_slave_set_state(OPEN_LIN_SLAVE_PID_RX);
                    break;
				}				
			}

			case (OPEN_LIN_SLAVE_PID_RX):
			{
				if (open_lin_data_layer_parity(rx_byte) == rx_byte)
				{
					open_lin_data_layer_frame.pid = (open_lin_pid_t)(rx_byte & OPEN_LIN_ID_MASK);
				} else
				{
					open_lin_error_handler(OPEN_LIN_SLAVE_ERROR_PID_PARITY);
					open_lin_slave_reset();
					break;
				}

				if (is_event_pid(open_lin_data_layer_frame.pid) == l_true)
				{
					if (open_lin_slave_event_response(open_lin_data_layer_frame.pid) == l_true)
					{
						/* open_lin_slave_poll() ends it once the response is out */
						open_lin_slave_set_state(OPEN_LIN_SLAVE_DATA_TX);
					} else
					{
						open_lin_slave_reset();
					}
					break;
				}

				if (open_lin_slave_set_lin_frame(&open_lin_data_layer_frame) == true)
				{
					if (open_lin_slave_state == OPEN_LIN_SLAVE_DATA_TX)
					{
						open_lin_set_rx_enabled(l_false); // Prevent receiving data transmitted by itself(Slave)
						open_lin_net_tx_handler(open_lin_data_layer_frame.pid);

						/* Section 2.7.3: fold the current response_error status into the
						 * designated frame's data, after the app fills it but before the
						 * checksum (which must cover this bit too) is computed. */
						if (s_response_error_frame_configured &&
						    (open_lin_data_layer_frame.pid == s_response_error_pid) &&
						    (s_response_error_byte_index < open_lin_data_layer_frame.length))
						{
							if (s_response_error)
							{
								open_lin_data_layer_frame.data_ptr[s_response_error_byte_index] |= (l_u8)(1u << s_response_error_bit_index);
							} else
							{
								open_lin_data_layer_frame.data_ptr[s_response_error_byte_index] &= (l_u8)~(1u << s_response_error_bit_index);
							}
						}

						open_lin_data_layer_frame.checksum = open_lin_data_layer_checksum(open_lin_data_layer_frame.pid, open_lin_data_layer_frame.checksum_type,
															open_lin_data_layer_frame.length, open_lin_data_layer_frame.data_ptr);
						if (slave_tx_start(open_lin_data_layer_frame.data_ptr, open_lin_data_layer_frame.length, open_lin_data_layer_frame.checksum) == l_false)
						{
							open_lin_error_handler(OPEN_LIN_SLAVE_ERROR_HW_TX);
							s_response_error = l_true; /* Section 2.7.3: error transmitting a frame this node owns */
							open_lin_slave_reset();
							break;
						}
						/* in the background from here: open_lin_slave_poll() ends it once the response is out */
						s_tx_kind = SLAVE_TX_FRAME;
						s_tx_pid = open_lin_data_layer_frame.pid;
						break;
					}
				} else
				{
					open_lin_error_handler(OPEN_LIN_SLAVE_ERROR_ID_NOT_FOUND);
					open_lin_slave_reset();
					break;
				}
				break;
			}
			case (OPEN_LIN_SLAVE_DATA_TX):
			{
				/* the response is going out in the background, nothing is to be received meanwhile */
				break;
			}

			case (OPEN_LIN_SLAVE_DATA_RX):
			{
				if (open_lin_slave_state_data_count < open_lin_data_layer_frame.length)
				{
					open_lin_slave_data_buff[open_lin_slave_state_data_count] = rx_byte;
					open_lin_slave_state_data_count ++;
				} else
				{
					/* checksum calculation */
					if (rx_byte == open_lin_data_layer_checksum(open_lin_data_layer_frame.pid, open_lin_data_layer_frame.checksum_type,
							open_lin_data_layer_frame.length, open_lin_slave_data_buff))
					{
						/* valid checksum, copy frame from internal buffer*/
						open_lin_memcpy(open_lin_data_layer_frame.data_ptr, open_lin_slave_data_buff, open_lin_data_layer_frame.length);
						open_lin_net_rx_handler(open_lin_data_layer_frame.pid);
					} else
					{
						open_lin_error_handler(OPEN_LIN_SLAVE_ERROR_INVALID_CHECKSUM);
						s_response_error = l_true; /* Section 2.7.3: error receiving a frame this node owns */
					}
					open_lin_slave_reset();
					break;
				}
				break;
			}
			default:
			{
				open_lin_error_handler(OPEN_LIN_SLAVE_ERROR_INVALID_DATA_RX);
				open_lin_slave_reset();
				break;
			}
		}
	}
}

void open_lin_slave_poll(void)
{
	l_bool sent;

	if (s_tx_kind == SLAVE_TX_NONE)
	{
		return;
	}

	if (open_lin_hw_tx_is_done() == l_true)
	{
		sent = l_true;
	} else if ((open_lin_hw_get_time_us() - s_tx_started_us) > OPEN_LIN_SLAVE_TX_TIMEOUT_US)
	{
		sent = l_false;
	} else
	{
		return; /* still going out */
	}

	if (s_tx_kind == SLAVE_TX_FRAME)
	{
		if (sent == l_true)
		{
			if (s_response_error_frame_configured && (s_tx_pid == s_response_error_pid))
			{
				s_response_error = l_false; /* Section 2.7.3: cleared once reported successfully */
			}
			{
				t_slave_event_entry *sent_entry = event_by_associated_pid(s_tx_pid);
				if (sent_entry != (t_slave_event_entry *)0)
				{
					sent_entry->updated = l_false; /* Section 2.3.3.2: sent as an ordinary frame, the signal is no longer updated */
				}
			}
		} else
		{
			open_lin_error_handler(OPEN_LIN_SLAVE_ERROR_HW_TX);
			s_response_error = l_true; /* Section 2.7.3: error transmitting a frame this node owns */
		}
	} else
	{
		if (sent == l_false)
		{
			open_lin_error_handler(OPEN_LIN_SLAVE_ERROR_HW_TX); /* not counted in response_error, Section 2.7.2 */
		} else if (s_tx_garbled == l_true)
		{
			s_event_garble = l_false; /* a collision leaves the update pending */
		} else
		{
			s_tx_event->updated = l_false;
		}
	}

	open_lin_slave_reset();
}

void open_lin_slave_set_state_callback(void (*callback)(t_open_lin_slave_state new_state))
{
	open_lin_slave_state_callback = callback;
}

l_bool open_lin_slave_dl_send_wake_up(void)
{
	return open_lin_hw_tx_break();
}
