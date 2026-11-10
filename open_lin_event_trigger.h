/**
 * @file open_lin_event_trigger.h
 * @brief LIN event-triggered frame descriptor (LIN-Spec 2.2A Section 2.3.3.2 /
 *        LDF "Event_triggered_frames" block).
 *
 *        An event-triggered frame has no data of its own: the master sends a
 *        header carrying the event-triggered frame's own PID, and whichever
 *        slave has new data on one of the associated unconditional frames
 *        responds. Per 2.3.3.2 the first data byte of every associated frame
 *        is reserved for that frame's protected identifier, which is how the
 *        master tells the responders apart. If more than one candidate
 *        responds at once the response is garbled; the master then runs the
 *        collision resolving schedule table once and returns to the table
 *        it came from.
 */
#ifndef OPEN_LIN_EVENT_TRIGGER_H_
#define OPEN_LIN_EVENT_TRIGGER_H_

#include "open_lin_types.h"
#include "open_lin_network_layer.h"

struct open_lin_master_frame_table_item_s;

/**
 * @brief One event-triggered frame's candidates and collision resolving table.
 *        Per 2.3.3.2 the candidates must have equal length, the same checksum
 *        model, and different publishers, and each frame's data byte 0 holds
 *        its protected id. The collision table must list at least all
 *        candidates.
 */
typedef struct {
	const open_lin_frame_slot_t **candidates;
	l_u8 candidate_count;
	struct open_lin_master_frame_table_item_s *collision_table;
	l_u8 collision_table_size;
} open_lin_event_trigger_t;

#endif /* OPEN_LIN_EVENT_TRIGGER_H_ */
