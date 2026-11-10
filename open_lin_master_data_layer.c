/*
 * open_lin_master_data_layer.c
 *
 *  Created on: 30.01.2018
 *      Author: ay7vi2
 */

#include "open_lin_types.h"
#include "open_lin_data_layer.h"
#include "open_lin_network_layer.h"
#include "open_lin_hw.h"
#include "open_lin_master_data_layer.h"
#include "open_lin_transport_layer.h"

static t_open_lin_master_state lin_master_state;
static void (*lin_master_state_callback)(t_open_lin_master_state);

static l_u8 master_rx_count = 0;
static l_u8 master_table_index = 0;
static t_master_frame_table_item *master_frame_table;
static l_u8 master_frame_table_size = 0;
static t_master_frame_table_item *pending_table = (t_master_frame_table_item *)0;
static l_u8 pending_table_size = 0;

/* Collision resolving (Section 2.3.3.2): the table to return to, and the entry to continue with */
static l_bool collision_active = l_false;
static t_master_frame_table_item *resume_table = (t_master_frame_table_item *)0;
static l_u8 resume_table_size = 0;
static l_u8 resume_index = 0;

/* Diagnostic Spec 5.4: MRF/SRF schedule tables run between executions of the normal schedule */
#define DIAG_PHASE_NONE 0u
#define DIAG_PHASE_MRF  1u
#define DIAG_PHASE_SRF  2u
static t_master_frame_table_item *diag_mrf_table = (t_master_frame_table_item *)0;
static l_u8 diag_mrf_table_size = 0;
static t_master_frame_table_item *diag_srf_table = (t_master_frame_table_item *)0;
static l_u8 diag_srf_table_size = 0;
static l_u8 diag_phase = DIAG_PHASE_NONE;
static t_master_frame_table_item *diag_return_table = (t_master_frame_table_item *)0;
static l_u8 diag_return_table_size = 0;

static l_u32 frame_start_time_us = 0;
static l_u32 frame_slot_us_of_current = 0;
static l_bool master_network_asleep = l_false;

static void open_lin_master_goto_idle(l_bool next_item);
static void data_layer_next_item(void);
static t_master_frame_table_item* get_current_item(void);

static void open_lin_master_set_state(t_open_lin_master_state new_state)
{
	lin_master_state = new_state;
	if (lin_master_state_callback) {
		lin_master_state_callback(lin_master_state);
	}
}

static void open_lin_master_goto_idle(l_bool next_item)
{
	master_rx_count = 0;
	open_lin_master_set_state(OPEN_LIN_MASTER_IDLE);
	if (next_item)
	{
		data_layer_next_item();
	} else {
		/* do nothing */
	}

	open_lin_set_rx_enabled(l_false); // Prevent receiving data transmitted by itself(Master)
}


/* Section 2.3.3.2 rules for one event-triggered table entry */
static l_bool event_trigger_is_valid(const t_master_frame_table_item *item)
{
	const open_lin_event_trigger_t *event = item->event_trigger;
	l_u8 i;
	l_u8 j;

	if ((event->candidates == (const open_lin_frame_slot_t **)0) || (event->candidate_count == 0u) ||
	    (event->collision_table == (t_master_frame_table_item *)0) || (event->collision_table_size == 0u))
	{
		return l_false;
	}

	for (i = 0u; i < event->candidate_count; i++)
	{
		const open_lin_frame_slot_t *candidate = event->candidates[i];
		l_bool in_collision_table = l_false;

		if ((candidate == (const open_lin_frame_slot_t *)0) || (candidate->data_length == 0u) ||
		    (candidate->data_length != item->slot.data_length) ||                  /* equal length, which is also the response length */
		    (candidate->checksum_type != event->candidates[0]->checksum_type))     /* same checksum model */
		{
			return l_false;
		}
		for (j = 0u; j < event->collision_table_size; j++)
		{
			if (event->collision_table[j].slot.pid == candidate->pid)
			{
				in_collision_table = l_true;
			}
		}
		if (in_collision_table == l_false) /* collision resolving has to be able to poll every candidate */
		{
			return l_false;
		}
	}
	return l_true;
}

l_bool open_lin_master_dl_validate_table(const t_master_frame_table_item *p_table, l_u8 p_size)
{
	l_u8 i;

	for (i = 0u; i < p_size; i++)
	{
		if ((p_table[i].event_trigger != (const open_lin_event_trigger_t *)0) && (event_trigger_is_valid(&p_table[i]) == l_false))
		{
			open_lin_error_handler(OPEN_LIN_MASTER_ERROR_EVENT_CONFIG);
			return l_false;
		}
	}
	return l_true;
}

void open_lin_master_dl_init(t_master_frame_table_item *p_master_frame_table, l_u8 p_master_frame_table_size)
{
	master_frame_table = p_master_frame_table;
	/* a table that breaks the event-triggered frame rules is not run at all, the master stays silent */
	master_frame_table_size = (open_lin_master_dl_validate_table(p_master_frame_table, p_master_frame_table_size) == l_true) ? p_master_frame_table_size : 0u;
	pending_table = (t_master_frame_table_item *)0;
	collision_active = l_false;
	diag_phase = DIAG_PHASE_NONE;
	master_table_index = 0;
	open_lin_master_goto_idle(l_false);
}

void open_lin_master_dl_set_diagnostic_tables(t_master_frame_table_item *p_mrf_table, l_u8 p_mrf_size,
                                              t_master_frame_table_item *p_srf_table, l_u8 p_srf_size)
{
	diag_mrf_table = p_mrf_table;
	diag_mrf_table_size = p_mrf_size;
	diag_srf_table = p_srf_table;
	diag_srf_table_size = p_srf_size;
}

l_bool open_lin_master_dl_switch_schedule_table(t_master_frame_table_item *p_table, l_u8 p_size)
{
	if ((p_table == (t_master_frame_table_item *)0) || (p_size == 0u) || (open_lin_master_dl_validate_table(p_table, p_size) == l_false))
	{
		return l_false;
	}
	pending_table = p_table;
	pending_table_size = p_size;
	return l_true;
}

static t_master_frame_table_item* get_current_item(void)
{
	return &(master_frame_table[master_table_index]);
}

/* Diagnostic Spec 5.4.3.1: the normal schedule has just run through - if a request frame is waiting
 * to go out, run the MRF table once; else if a response is awaited, run the SRF table once. */
static void start_diagnostic_schedule_if_needed(void)
{
#ifdef OPEN_LIN_TRANSPORT_LAYER
	t_master_frame_table_item *table = (t_master_frame_table_item *)0;
	l_u8 size = 0u;
	l_u8 phase = DIAG_PHASE_NONE;

	if ((diag_mrf_table != (t_master_frame_table_item *)0) && (diag_mrf_table_size > 0u) && (open_lin_master_diag_tx_pending() == l_true))
	{
		table = diag_mrf_table;
		size = diag_mrf_table_size;
		phase = DIAG_PHASE_MRF;
	} else if ((diag_srf_table != (t_master_frame_table_item *)0) && (diag_srf_table_size > 0u) &&
	           (open_lin_master_diag_response_expected(open_lin_hw_get_time_us()) == l_true))
	{
		table = diag_srf_table;
		size = diag_srf_table_size;
		phase = DIAG_PHASE_SRF;
	}

	if (phase != DIAG_PHASE_NONE)
	{
		diag_return_table = master_frame_table;
		diag_return_table_size = master_frame_table_size;
		master_frame_table = table;
		master_frame_table_size = size;
		master_table_index = 0;
		diag_phase = phase;
	}
#endif
}

static void data_layer_next_item(void)
{

	if (master_table_index >= (l_u8)(master_frame_table_size - 1u))
	{
		if (collision_active == l_true)
		{
			/* collision resolving table processed once: back to the table it came from */
			master_frame_table = resume_table;
			master_frame_table_size = resume_table_size;
			master_table_index = resume_index;
			collision_active = l_false;
			if (master_table_index == 0u)
			{
				start_diagnostic_schedule_if_needed(); /* the collision happened on the last entry, so the normal schedule is now through */
			}
		} else if (diag_phase != DIAG_PHASE_NONE)
		{
			/* diagnostic schedule executed once: a normal schedule runs before the next diagnostic one */
			master_frame_table = diag_return_table;
			master_frame_table_size = diag_return_table_size;
			master_table_index = 0;
			diag_phase = DIAG_PHASE_NONE;
		} else
		{
			master_table_index = 0;
			start_diagnostic_schedule_if_needed();
		}
	} else {
		master_table_index ++;
	}
}

/* Section 2.3.3.2: switch to the event-triggered frame's collision resolving table, starting with the
 * next frame slot. Returns l_false (nothing changes) if one is already running or none is configured. */
static l_bool start_collision_resolving(const open_lin_event_trigger_t *event)
{
	if ((collision_active == l_true) || (event->collision_table == (t_master_frame_table_item *)0) || (event->collision_table_size == 0u))
	{
		return l_false;
	}
	resume_table = master_frame_table;
	resume_table_size = master_frame_table_size;
	resume_index = (master_table_index >= (l_u8)(master_frame_table_size - 1u)) ? 0u : (l_u8)(master_table_index + 1u);
	master_frame_table = event->collision_table;
	master_frame_table_size = event->collision_table_size;
	master_table_index = 0;
	collision_active = l_true;
	return l_true;
}

/* Garbled or incomplete response to an event-triggered header: more than one slave answered */
static void event_collision(const open_lin_event_trigger_t *event)
{
	open_lin_error_handler(OPEN_LIN_MASTER_ERROR_EVENT_COLLISION);
	open_lin_master_goto_idle(start_collision_resolving(event) ? l_false : l_true);
}

/* Valid response to an event-triggered header: the PID in data byte 0 says which associated frame it is */
static void event_frame_received(const open_lin_event_trigger_t *event, const l_u8 *data, l_u8 length)
{
	l_u8 i;
	for (i = 0u; (i < event->candidate_count) && (length > 0u); i++)
	{
		const open_lin_frame_slot_t *candidate = event->candidates[i];
		if (data[0] == open_lin_data_layer_parity(candidate->pid))
		{
			(void)open_lin_memcpy(candidate->data_ptr, data, length);
			open_lin_master_dl_rx_callback((open_lin_frame_slot_t *)candidate);
			return;
		}
	}
	open_lin_error_handler(OPEN_LIN_MASTER_ERROR_EVENT_UNKNOWN_PID);
}


l_bool open_lin_master_data_tx_header(open_lin_frame_slot_t  *slot)
{
	l_bool result = l_true;
	result = (open_lin_hw_tx_break()) && result;
	result = (open_lin_hw_tx_byte(0x55)) && result;
	result = (open_lin_hw_tx_byte(open_lin_data_layer_parity(slot->pid))) && result;
	return result;
}

l_bool open_lin_master_data_tx_data(open_lin_frame_slot_t *slot)
{
	l_u8 i = 0;
	l_bool result = l_true;
	for (i = 0; i < slot->data_length; i++)
	{
		result = (open_lin_hw_tx_byte(slot->data_ptr[i])) && result;
	}

	result = (open_lin_hw_tx_byte(open_lin_data_layer_checksum(slot->pid, slot->checksum_type, slot->data_length, slot->data_ptr))) && result;
	return result;
}

l_bool open_lin_master_dl_rx(l_u8 rx_byte)
{
	open_lin_frame_slot_t* slot = &(get_current_item()->slot);
	static l_u8 frame_tmp_buffer[OPEN_LIN_MAX_FRAME_LENGTH];
	l_bool pending = false;

	if (lin_master_state == OPEN_LIN_MASTER_DATA_RX)
	{
		if (master_rx_count < slot->data_length)
		{
			frame_tmp_buffer[master_rx_count] = rx_byte;
			master_rx_count ++;
			pending = l_true;
		} else
		{
			const open_lin_event_trigger_t *event = get_current_item()->event_trigger;
			if (rx_byte == open_lin_data_layer_checksum(slot->pid,slot->checksum_type,slot->data_length,frame_tmp_buffer))
			{
				if (event != (const open_lin_event_trigger_t *)0)
				{
					event_frame_received(event, frame_tmp_buffer, slot->data_length);
				} else
				{
					(void)open_lin_memcpy(slot->data_ptr,frame_tmp_buffer,slot->data_length);
#ifdef OPEN_LIN_TRANSPORT_LAYER
					if (slot->pid == OPEN_LIN_DIAG_RESPONSE)
					{
						open_lin_master_transport_layer_handle_rx(slot);
					} else
#endif
					{
						open_lin_master_dl_rx_callback(slot);
					}
				}
			} else if (event != (const open_lin_event_trigger_t *)0)
			{
				event_collision(event);
				return pending;
			} else
			{
				open_lin_error_handler(OPEN_LIN_MASTER_ERROR_CHECKSUM);
			}
			open_lin_master_goto_idle(l_true);
		}
	}
	return pending;
}


void open_lin_master_dl_set_asleep(l_bool asleep)
{
	master_network_asleep = asleep;
}

l_bool open_lin_master_dl_is_asleep(void)
{
	return master_network_asleep;
}

l_bool open_lin_master_dl_send_wake_up(void)
{
	return open_lin_hw_tx_break();
}

void open_lin_master_dl_handler()
{
	t_master_frame_table_item* master_table_item;

	/* Apply a requested table switch only between frames, so the slot in progress isn't disturbed */
	if ((pending_table != (t_master_frame_table_item *)0) && (lin_master_state == OPEN_LIN_MASTER_IDLE))
	{
		master_frame_table = pending_table;
		master_frame_table_size = pending_table_size;
		master_table_index = 0;
		pending_table = (t_master_frame_table_item *)0;
		collision_active = l_false; /* Section 2.3.3.2: switching tables before the collision is resolved cancels the resolving */
		diag_phase = DIAG_PHASE_NONE;
	}

	master_table_item = get_current_item();

	if (master_network_asleep == l_true)
	{
		return; /* network asleep, Section 2.6 - stay silent until open_lin_master_dl_set_asleep(l_false) */
	}

	if (master_frame_table_size > 0u)
	{
		if (lin_master_state == OPEN_LIN_MASTER_IDLE)
		{
			frame_start_time_us = open_lin_hw_get_time_us();
			frame_slot_us_of_current = master_table_item->frame_slot_us;

			if (open_lin_master_data_tx_header(&master_table_item->slot) == l_true)
			{
				if (master_table_item->slot.frame_type == OPEN_LIN_FRAME_TYPE_TRANSMIT)
				{
#ifdef OPEN_LIN_TRANSPORT_LAYER
					if (master_table_item->slot.pid == OPEN_LIN_DIAG_REQUEST)
					{
						/* fill slot.data_ptr with the next SF/FF/CF chunk before
						 * case OPEN_LIN_MASTER_TX_DATA below actually sends it */
						open_lin_master_transport_layer_handle_tx(&master_table_item->slot);
					}
#endif
					open_lin_master_set_state(OPEN_LIN_MASTER_TX_DATA);
				} else
				{
					open_lin_master_set_state(OPEN_LIN_MASTER_DATA_RX);
					master_rx_count = 0;
					open_lin_set_rx_enabled(l_true); // Ready for receiving from Slave
				}
			} else
			{
				open_lin_error_handler(OPEN_LIN_MASTER_ERROR_HEADER_TX);
				open_lin_master_set_state(OPEN_LIN_MASTER_IDLE);
			}
		} else
		{
			l_u32 frame_time_passed = open_lin_hw_get_time_us() - frame_start_time_us;
			if (frame_time_passed > master_table_item->frame_slot_us)
			{
				open_lin_error_handler(OPEN_LIN_MASTER_ERROR_FRAME_SLOT_TIMEOUT);
				open_lin_master_goto_idle(l_true);
				return;
			}
		}
		
		switch (lin_master_state)
		{
			case OPEN_LIN_MASTER_IDLE:
			{
				/* do nothing */
				break;
			}
			case OPEN_LIN_MASTER_DATA_RX:
			{
				l_u8 i = 0;
				l_u8 len = 1 + master_table_item->slot.data_length; // extra 1 bytes for checksum
				for (i = 0; i < len; i++) {
					const l_u32 read_timeout = 1000;
					l_bool frame_timeout = false;
					l_u8 bytes_read = 0;
					l_u8 rx_byte = 0;
					do {
						/* read into a local, not slot.data_ptr[i]: len goes one past
						 * data_length for the checksum byte, which would write out
						 * of bounds on a slot buffer sized to exactly data_length
						 * (e.g. diag_response_buffer[8] with data_length=8).
						 * open_lin_master_dl_rx() does its own buffering and only
						 * copies into slot.data_ptr once the checksum validates. */
						bytes_read = open_lin_hw_rx_byte(&rx_byte, read_timeout);
						frame_timeout = (open_lin_hw_get_time_us() - frame_start_time_us) > master_table_item->frame_slot_us;
					} while (bytes_read <= 0 && !frame_timeout);
					if (frame_timeout) {
						if (master_table_item->event_trigger != (const open_lin_event_trigger_t *)0)
						{
							if (i == 0u)
							{
								/* Section 2.3.3.2: no slave had new data - the slot just stays silent, not an error */
								open_lin_master_goto_idle(l_true);
							} else
							{
								event_collision(master_table_item->event_trigger);
							}
							return;
						}
						open_lin_error_handler(OPEN_LIN_MASTER_ERROR_FRAME_SLOT_TIMEOUT);
						open_lin_master_goto_idle(l_true);
						return;
					}
					else {
						open_lin_master_dl_rx(rx_byte);
					}
				}
				break;
			}

			case OPEN_LIN_MASTER_TX_DATA:
			{
				if (open_lin_master_data_tx_data(&master_table_item->slot) == l_true)
				{
					open_lin_master_goto_idle(l_true);
				} else
				{
					open_lin_error_handler(OPEN_LIN_MASTER_ERROR_DATA_TX);
					open_lin_master_set_state(OPEN_LIN_MASTER_IDLE);
				}
				break;
			}
			default:
				/* do nothing */
				break;
		}
	} else {
		/* empty master table do nothing */
	}
}

void open_lin_master_dl_set_state_callback(void (*callback)(t_open_lin_master_state new_state))
{
	lin_master_state_callback = callback;
}

l_u32 open_lin_master_dl_get_frame_start_time_us()
{
	return frame_start_time_us;
}

l_u32 open_lin_master_dl_get_frame_slot_us()
{
	return frame_slot_us_of_current;
}
