/**
 * @file open_lin_transport_layer.c
 * @brief LIN transport layer implementation
 */

#include "open_lin_transport_layer.h"
#include "open_lin_network_layer.h"
#include "open_lin_hw.h"

#ifdef OPEN_LIN_TRANSPORT_LAYER

extern open_lin_NAD_t open_lin_NAD;
extern l_u16 open_lin_supplier_id;
extern l_u16 open_lin_function_id;
extern l_u8  open_lin_variant_id;
extern open_lin_frame_slot_t *open_lin_assignable_frames[];
extern const l_u8 open_lin_assignable_frames_count;
/* LIN 2.0 compatibility (Section 2.5.1): message id per entry, 1:1 with
 * open_lin_assignable_frames[] above - Assign Frame Id identifies a frame by
 * this value, unlike Assign Frame ID Range's table index. */
extern const l_u16 open_lin_assignable_frame_message_ids[];
extern void open_lin_sid_callback(open_lin_SID_t sid);
extern void open_lin_slave_sleep_callback(void);
extern void open_lin_master_sleep_sent_callback(void);

#define OPEN_LIN_WILDCARD_SUPPLIER_ID 0x7FFFu /* Table 4.3 */
#define OPEN_LIN_WILDCARD_FUNCTION_ID 0xFFFFu /* Table 4.3 */

/* sentinel written to open_lin_frame_slot_t.pid to disable a frame (Table 4.16's
 * "unassign" value). Always distinguishable from a real id: open_lin_net_get_slot()
 * is only ever probed with parity-stripped 6-bit ids (0x00-0x3F, OPEN_LIN_ID_MASK),
 * so this can never accidentally match and the frame silently stops responding. */
#define OPEN_LIN_PID_UNASSIGNED 0xFFu

/* error_code for the "SID not present in open_lin_sid_handlers[]" case; the
 * spec leaves this value implementation-defined (Section 4.2.6.1's own example
 * uses 0x12 for "identifier not supported" on a per-service basis, not a
 * generic table) - 0x11 mirrors the common UDS "service not supported" code. */
#define OPEN_LIN_ERR_SERVICE_NOT_SUPPORTED 0x11u
#define OPEN_LIN_ERR_IDENTIFIER_NOT_SUPPORTED 0x12u /* Section 4.2.6.1 example */

/**
 * @brief Outcome of a SID handler, drives what open_lin_transport_layer_dispatch()
 *        sends back on 0x3D.
 */
typedef enum {
	OPEN_LIN_SID_RESULT_NO_RESPONSE = 0, /**< request not addressed to this node (id/NAD mismatch) - stay silent */
	OPEN_LIN_SID_RESULT_POSITIVE,        /**< resp_data[0..*resp_len) valid, respond RSID = SID + 0x40 */
	OPEN_LIN_SID_RESULT_NEGATIVE         /**< respond 0x7F, SID, *error_code, per Table 4.21 */
} t_open_lin_sid_result;

typedef t_open_lin_sid_result (*t_open_lin_sid_handler_fn)(const l_u8 *data, l_u8 *resp_data, l_u8 *resp_len, l_u8 *error_code);

typedef struct {
	open_lin_SID_t sid;
	t_open_lin_sid_handler_fn handler;
} t_open_lin_sid_handler_item;


/* static function definitions */
static t_open_lin_sid_result open_lin_assign_nad_handler(const l_u8 *data, l_u8 *resp_data, l_u8 *resp_len, l_u8 *error_code);
static t_open_lin_sid_result open_lin_assign_frame_id_handler(const l_u8 *data, l_u8 *resp_data, l_u8 *resp_len, l_u8 *error_code);
static t_open_lin_sid_result open_lin_read_by_id_handler(const l_u8 *data, l_u8 *resp_data, l_u8 *resp_len, l_u8 *error_code);
static t_open_lin_sid_result open_lin_assign_frame_id_range_handler(const l_u8 *data, l_u8 *resp_data, l_u8 *resp_len, l_u8 *error_code);
static l_bool open_lin_id_match(uint16_t val, uint16_t expected, uint16_t wildcard);
static l_bool open_lin_is_go_to_sleep_frame(const l_u8 *frame8);

/* static service handlers */
t_open_lin_sid_handler_item open_lin_sid_handlers[] = {
		{OPEN_LIN_SID_ASSIGN_NAD, open_lin_assign_nad_handler},
		{OPEN_LIN_SID_ASSIGN_FRAME_ID, open_lin_assign_frame_id_handler},
		{OPEN_LIN_SID_READ_BY_ID, open_lin_read_by_id_handler},
		{OPEN_LIN_SID_ASSIGN_FRAME_ID_RANGE, open_lin_assign_frame_id_range_handler}
};

/* transport layer engine state - one active session at a time, see Section 3.2.2 */
static t_open_lin_tp_rx_ctx s_tp_rx_ctx;
static t_open_lin_tp_tx_ctx s_tp_tx_ctx;

/* response message storage - t_open_lin_tp_tx_ctx only holds a pointer, so this
 * must outlive every open_lin_tp_tx_next() call for a multi-frame (FF+CF) response. */
static l_u8 s_tp_response_buf[OPEN_LIN_TP_MAX_MSG_LEN];

/* NAD to use for the pending response, captured from the request at dispatch time.
 * Must NOT be re-read from open_lin_NAD at tx time: services like Assign NAD change
 * open_lin_NAD as a side effect of handling the request, but Table 4.8 requires the
 * response to echo the *initial* (request) NAD, not the new one. */
static open_lin_NAD_t s_tp_response_nad;

/* Diagnostic Spec 5.6: the response has to start within P2max of the request (P2* after a response pending),
 * else the master has given up on it and a late response is discarded instead of sent. */
static l_u32 s_tp_response_ready_us = 0u;
static l_u32 s_tp_response_window_us = OPEN_LIN_MASTER_DIAG_P2_MAX_US;

/* master side: separate engine state, since a master originates requests on 0x3C
 * and reassembles responses on 0x3D - the mirror image of the slave role above.
 * Never both linked into the same binary (master.c and slave.c are separate
 * executables), so there's no risk of the two sessions interfering. */
static t_open_lin_tp_tx_ctx s_master_tp_tx_ctx;
static t_open_lin_tp_rx_ctx s_master_tp_rx_ctx;
static open_lin_NAD_t s_master_tp_request_nad;

/* debug/test only: set by open_lin_master_diag_request_debug_drop_cf(), consumed
 * by open_lin_master_transport_layer_handle_tx() right after the FF goes out -
 * see the header comment on that function. */
static l_bool s_master_tp_tx_debug_drop_cf = l_false;

/* set by open_lin_master_send_go_to_sleep(), consumed by
 * open_lin_master_transport_layer_handle_tx() - see the header comment on that function. */
static l_bool s_master_tp_tx_go_to_sleep_pending = l_false;

/* Section 5.4.2 (Diagnostic Spec): after the last request frame has gone out, the master polls
 * for the response until it is complete or P2max passes without it starting. */
static l_bool s_master_tp_response_expected = l_false;
static l_u32 s_master_tp_response_start_us = 0u;
static l_u32 s_master_tp_response_window_us = OPEN_LIN_MASTER_DIAG_P2_MAX_US; /* P2max, or P2* after a "response pending" */

static l_bool open_lin_id_match(uint16_t val, uint16_t expected, uint16_t wildcard)
{
	return (l_bool)((val == wildcard) || (val == expected));
}

/* go-to-sleep command, Section 2.6.3 / Table 2.1 and Table 4.4: NAD (the frame's
 * first data field) is the dedicated value 0x00 - reserved solely for this
 * command, distinct from the 0x7F broadcast NAD - with D1..D7 all 0xFF. */
static l_bool open_lin_is_go_to_sleep_frame(const l_u8 *frame8)
{
	l_u8 i;

	if (frame8[0] != OPEN_LIN_NAD_SLEEP)
	{
		return l_false;
	}
	for (i = 1u; i < 8u; i++)
	{
		if (frame8[i] != 0xFFu)
		{
			return l_false;
		}
	}
	return l_true;
}

/* assign NAD service implementation, Table 4.7 / Table 4.8
 * data[0..4] = D1..D5 = supplier id (LSB,MSB), function id (LSB,MSB), new NAD */
static t_open_lin_sid_result open_lin_assign_nad_handler(const l_u8 *data, l_u8 *resp_data, l_u8 *resp_len, l_u8 *error_code)
{
	uint16_t sup = data[0] | (((uint16_t)data[1]) << 8);
	uint16_t fun = data[2] | (((uint16_t)data[3]) << 8);

	(void)resp_data;
	(void)error_code;

	if (open_lin_id_match(sup, open_lin_supplier_id, OPEN_LIN_WILDCARD_SUPPLIER_ID) &&
	    open_lin_id_match(fun, open_lin_function_id, OPEN_LIN_WILDCARD_FUNCTION_ID))
	{
		open_lin_NAD = data[4];
		*resp_len = 0u; /* Table 4.8: positive response carries no data, RSID only */
		return OPEN_LIN_SID_RESULT_POSITIVE;
	}
	/* "A response shall only be sent if the NAD, the Supplier ID and the
	 * Function ID match." - Section 4.2.5.1 */
	return OPEN_LIN_SID_RESULT_NO_RESPONSE;
}

/* read by identifier service implementation, Table 4.18 / Table 4.19 / Table 4.20
 * data[0..4] = D1..D5 = identifier, supplier id (LSB,MSB), function id (LSB,MSB) */
static t_open_lin_sid_result open_lin_read_by_id_handler(const l_u8 *data, l_u8 *resp_data, l_u8 *resp_len, l_u8 *error_code)
{
	l_u8 id = data[0];
	uint16_t sup = data[1] | (((uint16_t)data[2]) << 8);
	uint16_t fun = data[3] | (((uint16_t)data[4]) << 8);

	if (!open_lin_id_match(sup, open_lin_supplier_id, OPEN_LIN_WILDCARD_SUPPLIER_ID) ||
	    !open_lin_id_match(fun, open_lin_function_id, OPEN_LIN_WILDCARD_FUNCTION_ID))
	{
		return OPEN_LIN_SID_RESULT_NO_RESPONSE; /* not addressed to this node */
	}

	if (id == 0u)
	{
		/* Table 4.20, id=0: LIN Product Identification - the only mandatory identifier */
		resp_data[0] = (l_u8)(open_lin_supplier_id & 0xFFu);
		resp_data[1] = (l_u8)(open_lin_supplier_id >> 8);
		resp_data[2] = (l_u8)(open_lin_function_id & 0xFFu);
		resp_data[3] = (l_u8)(open_lin_function_id >> 8);
		resp_data[4] = open_lin_variant_id;
		*resp_len = 5u;
		return OPEN_LIN_SID_RESULT_POSITIVE;
	}

	/* id=1 (serial number) and 32-63 (user defined) are optional and not
	 * implemented yet; 2-31 and 64-255 are reserved. */
	*error_code = OPEN_LIN_ERR_IDENTIFIER_NOT_SUPPORTED;
	return OPEN_LIN_SID_RESULT_NEGATIVE;
}

/* assign frame id service implementation - LIN 2.0 Diag and Config Spec,
 * Section 2.5.1 / Table 2.4 (superseded by Assign Frame ID Range in 2.1+,
 * kept here only so this master/slave can still configure/be configured by a
 * genuine LIN 2.0 node). data[0..4] = D1..D5 = supplier id (LSB,MSB), message
 * id (LSB,MSB), protected id. Unlike Assign NAD/Read By Id, Table 2.4's own
 * text says a response is sent "if the NAD and the Supplier ID match" - no
 * Function ID check. Frames with identifier 0x3C and up (diagnostic,
 * user-defined, reserved) can not be reassigned this way. */
static t_open_lin_sid_result open_lin_assign_frame_id_handler(const l_u8 *data, l_u8 *resp_data, l_u8 *resp_len, l_u8 *error_code)
{
	uint16_t sup = data[0] | (((uint16_t)data[1]) << 8);
	uint16_t msg_id = data[2] | (((uint16_t)data[3]) << 8);
	l_u8 raw_id = data[4] & OPEN_LIN_ID_MASK;
	l_u8 i;

	(void)resp_data;
	(void)error_code;

	if (!open_lin_id_match(sup, open_lin_supplier_id, OPEN_LIN_WILDCARD_SUPPLIER_ID))
	{
		return OPEN_LIN_SID_RESULT_NO_RESPONSE;
	}

	if (raw_id >= 0x3Cu)
	{
		return OPEN_LIN_SID_RESULT_NO_RESPONSE; /* "can not be changed" - Section 2.5.1 */
	}

	for (i = 0u; i < open_lin_assignable_frames_count; i++)
	{
		if (open_lin_assignable_frame_message_ids[i] == msg_id)
		{
			open_lin_assignable_frames[i]->pid = (open_lin_pid_t)raw_id;
			*resp_len = 0u; /* Table 2.5: positive response carries no data, RSID only */
			return OPEN_LIN_SID_RESULT_POSITIVE;
		}
	}

	return OPEN_LIN_SID_RESULT_NO_RESPONSE; /* message id not recognized */
}

/* assign frame id range service implementation, Table 4.16 / Table 4.17
 * data[0..4] = D1..D5 = start index, PID(idx)..PID(idx+3) as protected identifiers.
 * PID byte 0x00 = unassign, 0xFF = do not care, anything else = new protected PID. */
static t_open_lin_sid_result open_lin_assign_frame_id_range_handler(const l_u8 *data, l_u8 *resp_data, l_u8 *resp_len, l_u8 *error_code)
{
	l_u8 start = data[0];
	l_u8 i;

	(void)resp_data;
	(void)error_code;

	/* validate all four before applying any - Table 4.16 Example 2: an
	 * out-of-range slot is only fulfillable as "do not care" (0xFF). */
	for (i = 0u; i < 4u; i++)
	{
		l_u16 idx = (l_u16)start + i;
		if ((data[1u + i] != 0xFFu) && (idx >= open_lin_assignable_frames_count))
		{
			return OPEN_LIN_SID_RESULT_NO_RESPONSE; /* "the slave shall reject the request" */
		}
	}

	for (i = 0u; i < 4u; i++)
	{
		l_u8 raw = data[1u + i];
		if (raw == 0xFFu)
		{
			continue; /* do not care: leave this frame's PID unchanged */
		}
		open_lin_assignable_frames[start + i]->pid =
			(raw == 0x00u) ? (open_lin_pid_t)OPEN_LIN_PID_UNASSIGNED
			               : (open_lin_pid_t)(raw & OPEN_LIN_ID_MASK);
	}

	*resp_len = 0u; /* Table 4.17: positive response carries no data, RSID only */
	return OPEN_LIN_SID_RESULT_POSITIVE;
}

/* debug/test only: see open_lin_transport_layer_debug_response_pending(). The real response is held back
 * here while a "response pending" goes out first, and is queued for transmission once the delay is over. */
static l_u32 s_debug_pending_delay_us = 0u;
static l_u8 s_debug_final_buf[OPEN_LIN_TP_MAX_MSG_LEN];
static l_u16 s_debug_final_len = 0u;
static l_u32 s_debug_final_due_us = 0u;

void open_lin_transport_layer_debug_response_pending(l_u32 delay_us)
{
	s_debug_pending_delay_us = delay_us;
}

static void defer_response_if_debug(l_u8 sid)
{
	if ((s_debug_pending_delay_us == 0u) || (s_tp_tx_ctx.active == l_false))
	{
		return; /* not armed, or the service stayed silent */
	}
	s_debug_final_len = s_tp_tx_ctx.total_len;
	(void)open_lin_memcpy(s_debug_final_buf, s_tp_response_buf, s_debug_final_len);
	s_debug_final_due_us = open_lin_hw_get_time_us() + s_debug_pending_delay_us;
	s_debug_pending_delay_us = 0u; /* one shot */

	s_tp_response_buf[0] = 0x7Fu;
	s_tp_response_buf[1] = sid;
	s_tp_response_buf[2] = 0x78u; /* response pending */
	open_lin_tp_tx_start(&s_tp_tx_ctx, s_tp_response_buf, 3u);
}

/**
 * @brief Dispatch one fully reassembled node-configuration message (SF, or
 *        FF+CF concatenated) to its SID handler.
 * @param[in] msg          msg[0] = SID, msg[1..len-1] = data bytes.
 * @param[in] len           total length including the SID byte.
 * @param[in] requester_nad NAD carried by the frame that completed the message.
 */
static void open_lin_transport_layer_dispatch(const l_u8 *msg, l_u16 len, open_lin_NAD_t requester_nad)
{
	uint8_t i = 0;

	if (len < 1u)
	{
		return; /* nothing to dispatch */
	}

	if ((requester_nad != open_lin_NAD) &&
	    (requester_nad != OPEN_LIN_NAD_BROADCAST) &&
	    (requester_nad != OPEN_LIN_NAD_FUNCTIONAL))
	{
		return; /* different NAD, not addressed to this node */
	}

	s_tp_response_ready_us = open_lin_hw_get_time_us(); /* the request is complete: P2max starts here */
	s_tp_response_window_us = OPEN_LIN_MASTER_DIAG_P2_MAX_US;

	for (i = 0; i < (sizeof(open_lin_sid_handlers)/sizeof(open_lin_sid_handlers[0])); i++)
	{
		if (open_lin_sid_handlers[i].sid == msg[0])
		{
			l_u8 resp_data[5];
			l_u8 resp_len = 0u;
			l_u8 error_code = 0u;
			t_open_lin_sid_result result = open_lin_sid_handlers[i].handler(&msg[1], resp_data, &resp_len, &error_code);

			s_tp_response_nad = requester_nad; /* echo the request's NAD, Table 4.8 */

			switch (result)
			{
				case OPEN_LIN_SID_RESULT_POSITIVE:
					s_tp_response_buf[0] = (l_u8)(msg[0] + 0x40u); /* RSID, Section 4.2.3.5 */
					(void)open_lin_memcpy(&s_tp_response_buf[1], resp_data, resp_len);
					open_lin_tp_tx_start(&s_tp_tx_ctx, s_tp_response_buf, (l_u16)(resp_len + 1u));
					break;

				case OPEN_LIN_SID_RESULT_NEGATIVE:
					s_tp_response_buf[0] = 0x7Fu;
					s_tp_response_buf[1] = msg[0];
					s_tp_response_buf[2] = error_code;
					open_lin_tp_tx_start(&s_tp_tx_ctx, s_tp_response_buf, 3u);
					break;

				case OPEN_LIN_SID_RESULT_NO_RESPONSE:
				default:
					break; /* stay silent, nothing queued */
			}

			defer_response_if_debug(msg[0]);
			open_lin_sid_callback((open_lin_SID_t)msg[0]);
			return;
		}
	}

	/* SID not in open_lin_sid_handlers[]: negative response, Table 4.21 */
	s_tp_response_nad = requester_nad;
	s_tp_response_buf[0] = 0x7Fu;
	s_tp_response_buf[1] = msg[0];
	s_tp_response_buf[2] = OPEN_LIN_ERR_SERVICE_NOT_SUPPORTED;
	open_lin_tp_tx_start(&s_tp_tx_ctx, s_tp_response_buf, 3u);
}

void open_lin_tp_rx_init(t_open_lin_tp_rx_ctx *ctx)
{
	ctx->active = l_false;
	ctx->total_len = 0u;
	ctx->received_len = 0u;
	ctx->next_sn = 1u;
	ctx->last_frame_time_us = 0u;
}

l_bool open_lin_tp_rx_feed(t_open_lin_tp_rx_ctx *ctx, const l_u8 *frame8, open_lin_NAD_t own_nad, l_u32 now_us)
{
	open_lin_NAD_t nad = frame8[0];
	l_u8 pci_type = (l_u8)((frame8[1] >> 4) & 0x0Fu);
	l_bool addressed = (nad == own_nad) || (nad == OPEN_LIN_NAD_BROADCAST) || (nad == OPEN_LIN_NAD_FUNCTIONAL);

	if (addressed == l_false)
	{
		return l_false; /* meant for another node, ignore without disturbing ctx */
	}

	switch (pci_type)
	{
		case OPEN_LIN_PCI_SF:
		{
			l_u8 len = (l_u8)(frame8[1] & 0x0Fu); /* SID + up to 5 data bytes */
			if ((len == 0u) || (len > 6u))
			{
				return l_false; /* Section 3.2.3: SF with length > 6 is ignored */
			}
			ctx->active = l_false; /* abort any reception in progress, start fresh */
			(void)open_lin_memcpy(ctx->buffer, &frame8[2], len);
			ctx->received_len = len;
			ctx->total_len = len;
			ctx->last_frame_time_us = now_us;
			return l_true; /* complete already */
		}

		case OPEN_LIN_PCI_FF:
		{
			l_u16 len = (l_u16)(((frame8[1] & 0x0Fu) << 8) | frame8[2]);
			if (len < 7u)
			{
				return l_false; /* Section 3.2.3: FF with length < 7 is ignored */
			}
			if (len > OPEN_LIN_TP_MAX_MSG_LEN)
			{
				ctx->active = l_false; /* can't buffer it, don't start reception */
				return l_false;
			}
			(void)open_lin_memcpy(ctx->buffer, &frame8[3], 5u); /* SID + D1..D4 */
			ctx->total_len = len;
			ctx->received_len = 5u;
			ctx->next_sn = 1u;
			ctx->active = l_true;
			ctx->last_frame_time_us = now_us;
			return l_false; /* not complete yet */
		}

		case OPEN_LIN_PCI_CF:
		{
			l_u8 sn = (l_u8)(frame8[1] & 0x0Fu);
			l_u16 remaining;
			l_u8 chunk;

			if (ctx->active == l_false)
			{
				return l_false; /* CF with nothing in progress: ignore */
			}
			if (sn != ctx->next_sn)
			{
				ctx->active = l_false; /* Section 3.2.3: unexpected SN aborts reception */
				return l_false;
			}

			remaining = (l_u16)(ctx->total_len - ctx->received_len);
			chunk = (remaining < 6u) ? (l_u8)remaining : 6u;
			(void)open_lin_memcpy(&ctx->buffer[ctx->received_len], &frame8[2], chunk);
			ctx->received_len = (l_u16)(ctx->received_len + chunk);
			ctx->next_sn = (ctx->next_sn == 15u) ? 0u : (l_u8)(ctx->next_sn + 1u);
			ctx->last_frame_time_us = now_us;

			if (ctx->received_len >= ctx->total_len)
			{
				ctx->active = l_false;
				return l_true;
			}
			return l_false;
		}

		default:
			return l_false; /* unexpected PCI type, ignored per Section 3.2.3 */
	}
}

l_bool open_lin_tp_rx_timed_out(const t_open_lin_tp_rx_ctx *ctx, l_u32 now_us)
{
	if (ctx->active == l_false)
	{
		return l_false;
	}
	return ((now_us - ctx->last_frame_time_us) > OPEN_LIN_TP_N_CR_TIMEOUT_US) ? l_true : l_false;
}

void open_lin_tp_tx_start(t_open_lin_tp_tx_ctx *ctx, const l_u8 *msg, l_u16 len)
{
	ctx->msg = msg;
	ctx->total_len = len;
	ctx->sent_len = 0u;
	ctx->next_sn = 1u;
	ctx->active = (len > 0u) ? l_true : l_false;
}

l_bool open_lin_tp_tx_next(t_open_lin_tp_tx_ctx *ctx, open_lin_NAD_t nad, l_u8 *out_frame8)
{
	l_u8 i;

	for (i = 0u; i < 8u; i++)
	{
		out_frame8[i] = 0xFFu;
	}

	if (ctx->active == l_false)
	{
		return l_false; /* nothing pending: caller gets an all-idle frame */
	}

	out_frame8[0] = nad;

	if ((ctx->sent_len == 0u) && (ctx->total_len <= 6u))
	{
		/* fits in a Single Frame */
		out_frame8[1] = (l_u8)((OPEN_LIN_PCI_SF << 4) | (ctx->total_len & 0x0Fu));
		(void)open_lin_memcpy(&out_frame8[2], ctx->msg, ctx->total_len);
		ctx->sent_len = ctx->total_len;
		ctx->active = l_false;
		return l_false;
	}

	if (ctx->sent_len == 0u)
	{
		/* First Frame: SID + D1..D4 (5 bytes), LEN carries the low 8 bits of length */
		out_frame8[1] = (l_u8)((OPEN_LIN_PCI_FF << 4) | ((ctx->total_len >> 8) & 0x0Fu));
		out_frame8[2] = (l_u8)(ctx->total_len & 0xFFu);
		(void)open_lin_memcpy(&out_frame8[3], ctx->msg, 5u);
		ctx->sent_len = 5u;
		return l_true;
	}

	/* Consecutive Frame: up to 6 more bytes */
	{
		l_u16 remaining = (l_u16)(ctx->total_len - ctx->sent_len);
		l_u8 chunk = (remaining < 6u) ? (l_u8)remaining : 6u;
		out_frame8[1] = (l_u8)((OPEN_LIN_PCI_CF << 4) | (ctx->next_sn & 0x0Fu));
		(void)open_lin_memcpy(&out_frame8[2], &ctx->msg[ctx->sent_len], chunk);
		ctx->sent_len = (l_u16)(ctx->sent_len + chunk);
		ctx->next_sn = (ctx->next_sn == 15u) ? 0u : (l_u8)(ctx->next_sn + 1u);
	}

	if (ctx->sent_len >= ctx->total_len)
	{
		ctx->active = l_false;
		return l_false;
	}
	return l_true;
}

void open_lin_transport_layer_poll(l_u32 now_us)
{
	if ((s_debug_final_len > 0u) && (s_tp_tx_ctx.active == l_false) && ((l_u32)(now_us - s_debug_final_due_us) < 0x80000000u))
	{
		/* the delay is over and the "response pending" has gone out: now the real response */
		(void)open_lin_memcpy(s_tp_response_buf, s_debug_final_buf, s_debug_final_len);
		open_lin_tp_tx_start(&s_tp_tx_ctx, s_tp_response_buf, s_debug_final_len);
		s_debug_final_len = 0u;
		s_tp_response_ready_us = now_us; /* after a response pending the master allows P2* for the real one */
		s_tp_response_window_us = OPEN_LIN_MASTER_DIAG_P2_STAR_US;
	}

	if (open_lin_tp_rx_timed_out(&s_tp_rx_ctx, now_us) == l_true)
	{
		open_lin_tp_rx_init(&s_tp_rx_ctx); /* drop the stale partial message so the next SF/FF starts clean */
		open_lin_error_handler(OPEN_LIN_SLAVE_ERROR_TP_RX_TIMEOUT);
	}
}

void open_lin_master_transport_layer_poll(l_u32 now_us)
{
	if (open_lin_tp_rx_timed_out(&s_master_tp_rx_ctx, now_us) == l_true)
	{
		open_lin_tp_rx_init(&s_master_tp_rx_ctx);
		open_lin_error_handler(OPEN_LIN_MASTER_ERROR_TP_RX_TIMEOUT);
	}
}

void open_lin_transport_layer_handle(open_lin_frame_slot_t *slot)
{
	if ((slot == OPEN_LIN_NET_SLOT_EMPTY) || (slot->pid != OPEN_LIN_DIAG_REQUEST))
	{
		/* diagnostic request slot not registered by the app, or wrong slot - nothing to do */
		return;
	}

	if (open_lin_is_go_to_sleep_frame(slot->data_ptr) == l_true)
	{
		open_lin_slave_sleep_callback();
		return;
	}

	if (open_lin_tp_rx_feed(&s_tp_rx_ctx, slot->data_ptr, open_lin_NAD, open_lin_hw_get_time_us()) == l_true)
	{
		open_lin_transport_layer_dispatch(s_tp_rx_ctx.buffer, s_tp_rx_ctx.received_len, slot->data_ptr[0]);
	}
}

void open_lin_transport_layer_handle_tx(open_lin_frame_slot_t *slot)
{
	if ((slot == OPEN_LIN_NET_SLOT_EMPTY) || (slot->pid != OPEN_LIN_DIAG_RESPONSE))
	{
		/* diagnostic response slot not registered by the app, or wrong slot - nothing to do */
		return;
	}

	if ((s_tp_tx_ctx.active == l_true) && (s_tp_tx_ctx.sent_len == 0u) &&
	    ((l_u32)(open_lin_hw_get_time_us() - s_tp_response_ready_us) > s_tp_response_window_us))
	{
		/* Diagnostic Spec 5.6: too late, the master has stopped waiting for this response - drop it
		 * (this slot goes out idle) rather than have it show up as the answer to a later request. */
		s_tp_tx_ctx.active = l_false;
		open_lin_error_handler(OPEN_LIN_SLAVE_ERROR_RESPONSE_DISCARDED);
	}

	(void)open_lin_tp_tx_next(&s_tp_tx_ctx, s_tp_response_nad, slot->data_ptr);
}

l_bool open_lin_master_diag_request(open_lin_NAD_t nad, const l_u8 *msg, l_u16 len)
{
	if (s_master_tp_tx_ctx.active == l_true)
	{
		return l_false; /* a request is already in flight - one active session, Section 3.2.2 */
	}

	s_master_tp_request_nad = nad;
	s_master_tp_response_expected = l_false; /* a new request supersedes whatever was still being waited for */
	open_lin_tp_tx_start(&s_master_tp_tx_ctx, msg, len);
	return l_true;
}

l_bool open_lin_master_diag_tx_pending(void)
{
	return ((s_master_tp_tx_ctx.active == l_true) || (s_master_tp_tx_go_to_sleep_pending == l_true)) ? l_true : l_false;
}

l_bool open_lin_master_diag_response_expected(l_u32 now_us)
{
	if ((s_master_tp_response_expected == l_true) &&
	    (s_master_tp_rx_ctx.active == l_false) &&
	    ((l_u32)(now_us - s_master_tp_response_start_us) > s_master_tp_response_window_us))
	{
		s_master_tp_response_expected = l_false; /* P2max (P2* after a "response pending") passed without the response starting */
	}
	return s_master_tp_response_expected;
}

l_bool open_lin_master_diag_request_debug_drop_cf(open_lin_NAD_t nad, const l_u8 *msg, l_u16 len)
{
	l_bool result = open_lin_master_diag_request(nad, msg, len);
	if (result == l_true)
	{
		s_master_tp_tx_debug_drop_cf = l_true;
	}
	return result;
}

l_bool open_lin_master_send_go_to_sleep(void)
{
	if (s_master_tp_tx_ctx.active == l_true)
	{
		return l_false; /* a request is already in flight - one active session, Section 3.2.2 */
	}

	s_master_tp_tx_go_to_sleep_pending = l_true;
	return l_true;
}

void open_lin_master_transport_layer_handle_tx(open_lin_frame_slot_t *slot)
{
	l_bool more;
	l_bool was_sending;
	l_u8 i;

	if ((slot == OPEN_LIN_NET_SLOT_EMPTY) || (slot->pid != OPEN_LIN_DIAG_REQUEST))
	{
		return;
	}

	if (s_master_tp_tx_go_to_sleep_pending == l_true)
	{
		slot->data_ptr[0] = OPEN_LIN_NAD_SLEEP;
		for (i = 1u; i < 8u; i++)
		{
			slot->data_ptr[i] = 0xFFu;
		}
		s_master_tp_tx_go_to_sleep_pending = l_false;
		open_lin_master_sleep_sent_callback();
		return;
	}

	was_sending = s_master_tp_tx_ctx.active;
	more = open_lin_tp_tx_next(&s_master_tp_tx_ctx, s_master_tp_request_nad, slot->data_ptr);

	if ((was_sending == l_true) && (more == l_false) && (s_master_tp_request_nad != OPEN_LIN_NAD_FUNCTIONAL))
	{
		/* last frame of the request is going out: from here the slave's response is awaited */
		s_master_tp_response_expected = l_true;
		s_master_tp_response_window_us = OPEN_LIN_MASTER_DIAG_P2_MAX_US;
		s_master_tp_response_start_us = open_lin_hw_get_time_us();
	}

	if ((s_master_tp_tx_debug_drop_cf == l_true) && (more == l_true))
	{
		/* `more == l_true` here means the frame just filled in was the FF (a
		 * lone SF always returns l_false) - stop before any CF goes out, so
		 * the target is left waiting on a Consecutive Frame that never comes. */
		s_master_tp_tx_ctx.active = l_false;
		s_master_tp_tx_debug_drop_cf = l_false;
	}
}

void open_lin_master_transport_layer_handle_rx(open_lin_frame_slot_t *slot)
{
	open_lin_NAD_t expected_nad;

	if ((slot == OPEN_LIN_NET_SLOT_EMPTY) || (slot->pid != OPEN_LIN_DIAG_RESPONSE))
	{
		return;
	}

	expected_nad = s_master_tp_request_nad;
	if ((expected_nad == OPEN_LIN_NAD_BROADCAST) || (expected_nad == OPEN_LIN_NAD_FUNCTIONAL))
	{
		/* the request went out wildcard-addressed, so the response carries the
		 * responding slave's own initial NAD instead (Table 4.8) - not knowable
		 * in advance, so accept whatever NAD this response actually carries. */
		expected_nad = slot->data_ptr[0];
	}

	if (open_lin_tp_rx_feed(&s_master_tp_rx_ctx, slot->data_ptr, expected_nad, open_lin_hw_get_time_us()) == l_true)
	{
		if ((s_master_tp_rx_ctx.received_len == 3u) && (s_master_tp_rx_ctx.buffer[0] == 0x7Fu) &&
		    (s_master_tp_rx_ctx.buffer[2] == OPEN_LIN_NRC_RESPONSE_PENDING))
		{
			/* Diagnostic Spec 5.6: not the final response - the slave needs more time. Keep waiting, now for
			 * up to P2* from this frame, and don't hand the application a response that isn't one. */
			s_master_tp_response_window_us = OPEN_LIN_MASTER_DIAG_P2_STAR_US;
			s_master_tp_response_start_us = open_lin_hw_get_time_us();
			return;
		}
		s_master_tp_response_expected = l_false; /* response complete */
		open_lin_master_diag_response_callback(s_master_tp_rx_ctx.buffer, s_master_tp_rx_ctx.received_len);
	}
}

#endif
